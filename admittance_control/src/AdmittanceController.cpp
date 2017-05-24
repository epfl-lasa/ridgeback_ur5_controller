#include "AdmittanceController.h"

AdmittanceController::AdmittanceController(ros::NodeHandle &n,
                             double frequency,
                             std::string cmd_topic_platform,
                             std::string state_topic_platform,
                             std::string cmd_topic_arm,
                             std::string topic_arm_twist_world,
                             std::string topic_wrench_u_e,
                             std::string topic_wrench_u_c,
                             std::string state_topic_arm,
                             std::string wrench_topic,
                             std::string wrench_control_topic,
                             std::string laser_front_topic,
                             std::string laser_rear_topic,
                             std::vector<double> M_p,
                             std::vector<double> M_a,
                             std::vector<double> D,
                             std::vector<double> D_p,
                             std::vector<double> D_a,
                             std::vector<double> K,
                             std::vector<double> d_e,
                             double wrench_filter_factor,
                             double force_dead_zone_thres,
                             double torque_dead_zone_thres,
                             double obs_distance_thres,
                             double self_detect_thres) :
                             nh_(n), loop_rate_(frequency),
                             M_p_(M_p.data()), M_a_(M_a.data()), D_(D.data()),
                             D_p_(D_p.data()), D_a_(D_a.data()), K_(K.data()),
                             d_e_(d_e.data()),
                             obs_distance_thres_(obs_distance_thres),
                             self_detect_thres_(self_detect_thres),
                             wrench_filter_factor_(wrench_filter_factor),
                             force_dead_zone_thres_(force_dead_zone_thres),
                             torque_dead_zone_thres_(torque_dead_zone_thres) {
  platform_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_topic_platform, 5);
  platform_sub_ = nh_.subscribe(state_topic_platform, 5,
                          &AdmittanceController::state_platform_callback, this,
                          ros::TransportHints().reliable().tcpNoDelay());
  wrench_sub_ = nh_.subscribe(wrench_topic, 5,
                          &AdmittanceController::wrench_callback, this,
                          ros::TransportHints().reliable().tcpNoDelay());
  wrench_control_sub_ = nh_.subscribe(wrench_control_topic, 5,
                          &AdmittanceController::wrench_control_callback, this,
                          ros::TransportHints().reliable().tcpNoDelay());
  arm_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_topic_arm, 5);
  arm_pub_world_ = nh_.advertise<geometry_msgs::Twist>(
                                                    topic_arm_twist_world, 5);
  arm_sub_ = nh_.subscribe(state_topic_arm, 10,
                          &AdmittanceController::state_arm_callback, this,
                          ros::TransportHints().reliable().tcpNoDelay());
  wrench_pub_u_e_ = nh_.advertise<geometry_msgs::WrenchStamped>(
                                                    topic_wrench_u_e, 5);
  wrench_pub_u_c_ = nh_.advertise<geometry_msgs::WrenchStamped>(
                                                    topic_wrench_u_c, 5);
  laser_front_sub_ = nh_.subscribe(laser_front_topic, 1,
                          &AdmittanceController::laser_front_callback, this,
                          ros::TransportHints().reliable().tcpNoDelay());
  laser_rear_sub_ = nh_.subscribe(laser_rear_topic, 1,
                          &AdmittanceController::laser_rear_callback, this,
                          ros::TransportHints().reliable().tcpNoDelay());

  obs_pub_ = nh_.advertise<geometry_msgs::PointStamped>("obstacles", 5);

  ft_arm_ready_ = false;
  arm_world_ready_ = false;
  base_world_ready_ = false;
  world_arm_ready_ = false;

  init_TF();

  u_e_.setZero();
  u_c_.setZero();

  // Kinematic constraints between base and arm at the equilibrium
  kin_constraints_.setZero();
  kin_constraints_.topLeftCorner(3, 3).setIdentity();
  kin_constraints_.bottomRightCorner(3, 3).setIdentity();
  // Screw due to the torque
  kin_constraints_.topRightCorner(3, 3) << 0, -d_e_(2), d_e_(1),
                                           d_e_(2), 0, -d_e_(0),
                                           -d_e_(1), d_e_(0), 0;
  obs_vector_.setZero();
  laser_front_cloud_.points.resize(0);
  laser_rear_cloud_.points.resize(0);
}

//////////////////////
/// INITIALIZATION ///
//////////////////////
void AdmittanceController::init_TF() {
  tf::TransformListener listener;
  Matrix6d rot_matrix;
  rotation_base_.setZero();

  // Makes sure all TFs exists before enabling all transformations in the callbacks
  while (!get_rotation_matrix(rotation_base_, listener,
                                           "base_link", "ur5_arm_base_link")) {
    sleep(1);
  }

  while (!get_rotation_matrix(rot_matrix, listener,
                                          "world", "base_link")) {
    sleep(1);
  }
  base_world_ready_ = true;

  while (!get_rotation_matrix(rot_matrix, listener,
                                         "world", "ur5_arm_base_link")) {
    sleep(1);
  }
  arm_world_ready_ = true;
  while (!get_rotation_matrix(rot_matrix, listener,
                                         "ur5_arm_base_link", "world")) {
    sleep(1);
  }
  world_arm_ready_ = true;

  while (!get_rotation_matrix(rot_matrix, listener,
                                         "ur5_arm_base_link", "FT300_link")) {
    sleep(1);
  }
  ft_arm_ready_ = true;
}

////////////////////
/// CONTROL LOOP ///
////////////////////
void AdmittanceController::run() {
  // Desired twists
  Vector6d desired_twist_arm;
  Vector6d desired_twist_platform;
  geometry_msgs::Twist platform_twist_cmd;
  geometry_msgs::Twist arm_twist_cmd;
  geometry_msgs::Twist arm_twist_world;
  geometry_msgs::PointStamped obs_pub_msg;

  // Init integrator
  desired_twist_arm.setZero();
  desired_twist_platform.setZero();

  geometry_msgs::WrenchStamped wrench_u_e_;
  geometry_msgs::WrenchStamped wrench_u_c_;

  while(nh_.ok()) {
    // Dynamics computation
    compute_admittance(desired_twist_platform, desired_twist_arm,
                       loop_rate_.expectedCycleTime());

    // Obstacle avoidance for the platform
    avoid_obstacles(desired_twist_platform);

    // Copy commands to messages
    platform_twist_cmd.linear.x = desired_twist_platform(0);
    platform_twist_cmd.linear.y  = desired_twist_platform(1);
    platform_twist_cmd.linear.z = desired_twist_platform(2);
    platform_twist_cmd.angular.x = desired_twist_platform(3);
    platform_twist_cmd.angular.y = desired_twist_platform(4);
    platform_twist_cmd.angular.z = desired_twist_platform(5);

    arm_twist_cmd.linear.x = desired_twist_arm(0);
    arm_twist_cmd.linear.y = desired_twist_arm(1);
    arm_twist_cmd.linear.z = desired_twist_arm(2);
    arm_twist_cmd.angular.x = desired_twist_arm(3);
    arm_twist_cmd.angular.y = desired_twist_arm(4);
    arm_twist_cmd.angular.z = desired_twist_arm(5);

    arm_twist_world.linear.x  = twist_arm_world_frame_(0);
    arm_twist_world.linear.y  = twist_arm_world_frame_(1);
    arm_twist_world.linear.z  = twist_arm_world_frame_(2);
    arm_twist_world.angular.x = twist_arm_world_frame_(3);
    arm_twist_world.angular.y = twist_arm_world_frame_(4);
    arm_twist_world.angular.z = twist_arm_world_frame_(5);

    platform_pub_.publish(platform_twist_cmd);
    arm_pub_.publish(arm_twist_cmd);
    arm_pub_world_.publish(arm_twist_world);

    // publishing useful visualization for debugging
    wrench_u_e_.header.stamp = ros::Time::now();
    wrench_u_e_.header.frame_id = "ur5_arm_base_link";
    wrench_u_e_.wrench.force.x = u_e_(0);
    wrench_u_e_.wrench.force.y = u_e_(1);
    wrench_u_e_.wrench.force.z = u_e_(2);
    wrench_u_e_.wrench.torque.x = u_e_(3);
    wrench_u_e_.wrench.torque.y = u_e_(4);
    wrench_u_e_.wrench.torque.z = u_e_(5);
    wrench_pub_u_e_.publish(wrench_u_e_);

    wrench_u_c_.header.stamp = ros::Time::now();
    wrench_u_c_.header.frame_id = "ur5_arm_base_link";
    wrench_u_c_.wrench.force.x = u_c_(0);
    wrench_u_c_.wrench.force.y = u_c_(1);
    wrench_u_c_.wrench.force.z = u_c_(2);
    wrench_u_c_.wrench.torque.x = u_c_(3);
    wrench_u_c_.wrench.torque.y = u_c_(4);
    wrench_u_c_.wrench.torque.z = u_c_(5);
    wrench_pub_u_c_.publish(wrench_u_c_);

    obs_pub_msg.header.stamp = ros::Time::now();
    obs_pub_msg.header.frame_id = "base_link";
    obs_pub_msg.point.x = obs_vector_(0);
    obs_pub_msg.point.y = obs_vector_(1);
    obs_pub_msg.point.z = obs_vector_(2);
    obs_pub_.publish(obs_pub_msg);

    // For obstacle avoidance
    update_obstacles();

    ros::spinOnce();
    loop_rate_.sleep();
  }
}

// Admittance dynamics.
void AdmittanceController::compute_admittance(Vector6d &desired_twist_platform,
                                            Vector6d &desired_twist_arm,
                                            ros::Duration duration) {
  Vector6d x_ddot_p, x_ddot_a;

  // Translation
  x_ddot_p = M_p_.inverse()*(- D_p_ * desired_twist_platform 
                       + rotation_base_* kin_constraints_ *
                       (D_ * x_dot_a_ + K_ * (x_a_ - d_e_)));
  x_ddot_a = M_a_.inverse()*( - (D_ + D_a_) *(desired_twist_arm)
                                - K_ * (x_a_ - d_e_) + u_e_ + u_c_);

  // TODO: Rotation
  //Eigen::AngleAxisd quat_arm * quat_des.inverse()

  // Integrate for velocity based interface
  desired_twist_platform = desired_twist_platform + x_ddot_p * duration.toSec();
  desired_twist_arm = desired_twist_arm + x_ddot_a * duration.toSec();

  //std::cout << "Desired twist arm: " << desired_twist_arm << std::endl;
  //std::cout << "Desired twist platform: " << desired_twist_platform << std::endl;
}

/////////////////
/// CALLBACKS ///
/////////////////
void AdmittanceController::state_platform_callback(
    const nav_msgs::OdometryConstPtr msg) {
  x_p_ << msg->pose.pose.position.x, msg->pose.pose.position.y,
        msg->pose.pose.position.z, 0, 0, 0;
  tf::Quaternion q(msg->pose.pose.orientation.x,
                  msg->pose.pose.orientation.y,
                  msg->pose.pose.orientation.z,
                  msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(x_p_(3), x_p_(4), x_p_(5));

  x_dot_p_ << msg->twist.twist.linear.x, msg->twist.twist.linear.y,
    msg->twist.twist.linear.z, msg->twist.twist.angular.x,
    msg->twist.twist.angular.y, msg->twist.twist.angular.z;
}

void AdmittanceController::state_arm_callback(
    const cartesian_state_msgs::PoseTwistConstPtr msg) {
  x_a_ << msg->pose.position.x, msg->pose.position.y,
          msg->pose.position.z, 0, 0, 0;
  tf::Quaternion q(msg->pose.orientation.x,
                   msg->pose.orientation.y, msg->pose.orientation.z,
                   msg->pose.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(x_a_(3), x_a_(4), x_a_(5));
  x_a_(3) = wrap_angle(x_a_(3));
  x_a_(4) = wrap_angle(x_a_(4));
  x_a_(5) = wrap_angle(x_a_(5));

  x_dot_a_ << msg->twist.linear.x, msg->twist.linear.y,
          msg->twist.linear.z, msg->twist.angular.x, msg->twist.angular.y,
          msg->twist.angular.z;

  // Arm twist in the world frame
  get_arm_twist_world(twist_arm_world_frame_, listener_arm_);
}

void AdmittanceController::wrench_callback(
    const geometry_msgs::WrenchStampedConstPtr msg) {
    // Get transform from arm base link to platform base link
  Vector6d wrench_ft_frame;
  Matrix6d rotation_ft_base;
  if (ft_arm_ready_) {
    get_rotation_matrix(rotation_ft_base, listener_ft_,
                         "ur5_arm_base_link", "robotiq_force_torque_frame_id");

    wrench_ft_frame << msg->wrench.force.x, msg->wrench.force.y, 
                    msg->wrench.force.z, msg->wrench.torque.x, 
                    msg->wrench.torque.y, msg->wrench.torque.z;

    // Dead zone for the FT sensor
    if (wrench_ft_frame.topRows(3).norm() < force_dead_zone_thres_) {
      wrench_ft_frame.topRows(3).setZero();
    }
    if (wrench_ft_frame.bottomRows(3).norm() < torque_dead_zone_thres_) {
      wrench_ft_frame.bottomRows(3).setZero();
    }

    // Filter and update
    u_e_ <<  (1 - wrench_filter_factor_) * u_e_ +
         wrench_filter_factor_ * rotation_ft_base * wrench_ft_frame;
  }
}

void AdmittanceController::wrench_control_callback(
    const geometry_msgs::WrenchStampedConstPtr msg) {
  Vector6d wrench_control_world_frame;
  Matrix6d rotation_world_base;
  if (world_arm_ready_) {
    get_rotation_matrix(rotation_world_base, listener_control_,
                                             "ur5_arm_base_link", "world");
    wrench_control_world_frame << msg->wrench.force.x, msg->wrench.force.y,
          msg->wrench.force.z,  msg->wrench.torque.x, msg->wrench.torque.y,
          msg->wrench.torque.z;
    u_c_ << rotation_world_base * wrench_control_world_frame;
  }
}

void AdmittanceController::laser_front_callback(
        const sensor_msgs::LaserScanPtr msg) {
  listener_laser_front_.waitForTransform("/base_link", msg->header.frame_id,
                                         msg->header.stamp, ros::Duration(1.0));
  if (base_world_ready_) {
    projector_.transformLaserScanToPointCloud("base_link", *msg,
                                              laser_front_cloud_,
                                              listener_laser_front_, -1.0,
                                      laser_geometry::channel_option::Intensity);
  }
}

void AdmittanceController::laser_rear_callback(
        const sensor_msgs::LaserScanPtr msg) {
  listener_laser_rear_.waitForTransform("/base_link", msg->header.frame_id,
                                         msg->header.stamp, ros::Duration(1.0));
  if (base_world_ready_) {
    projector_.transformLaserScanToPointCloud("base_link", *msg,
                                              laser_rear_cloud_,
                                              listener_laser_rear_, -1.0,
                                      laser_geometry::channel_option::Intensity);
  }
}

//////////////////////////
/// OBSTACLE AVOIDANCE ///
//////////////////////////
void AdmittanceController::avoid_obstacles(Vector6d &desired_twist_platform) {
  // Assumption: if obs_vector = (0,0,0) there is no obstacle
  double hard_threshold = obs_distance_thres_ / 2.0;
  if (obs_vector_.norm() > 0.01) {
    Vector3d closest_point = get_closest_point_on_platform(obs_vector_);
    Vector3d obs_closest_point_frame = obs_vector_ - closest_point;
    obs_closest_point_frame(2) = 0.0;
    // If there is a component of the velocity driving the platform to the
    // obstacle ...
    if (desired_twist_platform.topRows(3).dot(obs_closest_point_frame) > 0.0) {
        // ... remove it lineary until fully removed at hard_threshold
        double factor = (1.0 - ((obs_closest_point_frame.norm()
                                - (obs_distance_thres_ - hard_threshold))
                                / hard_threshold));
        factor = std::min(std::max(factor, 0.0), 1.0);
        desired_twist_platform.topRows(3) = desired_twist_platform.topRows(3)
            - factor *
            (desired_twist_platform.topRows(3).dot(obs_closest_point_frame)
            * obs_closest_point_frame / obs_closest_point_frame.squaredNorm());
    }
  }
}

Vector3d AdmittanceController::get_closest_point_on_platform(Vector3d obstacle) {
  Vector3d closest_point;
  // The dimensions of the ridgeback are 0.96 m x 0.793 m and
  // base_link is placed in the middle
  closest_point(0) = ((obstacle(0) > 0) - (obstacle(0) < 0)) //sign
                                  * std::min(fabs(obstacle(0)), 0.48);
  closest_point(1) = ((obstacle(1) > 0) - (obstacle(1) < 0)) //sign
                                  * std::min(fabs(obstacle(1)), 0.4);
  closest_point(2) = obstacle(2);

  return closest_point;
}

void AdmittanceController::update_obstacles() {
  Eigen::Vector3d sum_vectors;
  int n_vectors = 0;
  sum_vectors.setZero();

  for (unsigned int i=0 ; i < laser_front_cloud_.points.size() ; i ++) {
    Eigen::Vector3d cur_vector_front;
    cur_vector_front << laser_front_cloud_.points.at(i).x,
      laser_front_cloud_.points.at(i).y,
      laser_front_cloud_.points.at(i).z;

    if (isObstacleMeasurement(cur_vector_front)) {
      sum_vectors = sum_vectors + cur_vector_front;
      n_vectors++;
    }
  }
  for (unsigned int i=0 ; i < laser_rear_cloud_.points.size() ; i ++) {
    Eigen::Vector3d cur_vector_rear;
    cur_vector_rear << laser_rear_cloud_.points.at(i).x,
      laser_rear_cloud_.points.at(i).y,
      laser_rear_cloud_.points.at(i).z;

    if (isObstacleMeasurement(cur_vector_rear)) {
      sum_vectors = sum_vectors + cur_vector_rear;
      n_vectors++;
    }
  }
  if (n_vectors > 0) {
    obs_vector_ = sum_vectors/n_vectors;
  } else {
    obs_vector_.setZero();
  }
}

bool AdmittanceController::isObstacleMeasurement(Vector3d &measurement) {
  // Consider only measurements that are between self_detect_thres_ and
  // obs_distance_thres_.
  // The dimensions of the ridgeback are 0.96 m x 0.793 m and
  // base_link is placed in the middle
  return (std::abs(measurement(0)) < (0.48 + obs_distance_thres_) &&
           std::abs(measurement(1)) < (0.4 + obs_distance_thres_)) &&
         (std::abs(measurement(0)) > (0.48 + self_detect_thres_) ||
            std::abs(measurement(1)) > (0.4 + self_detect_thres_));
}


////////////
/// UTIL ///
////////////
void AdmittanceController::get_arm_twist_world(Vector6d &twist_arm_world_frame,
                                            tf::TransformListener & listener) {
  // publishing the cartesian velocity of the EE in the world-frame
  Matrix6d rotation_a_base_world;
  Matrix6d rotation_p_base_world;

  twist_arm_world_frame.setZero();

  if (arm_world_ready_ && base_world_ready_) {
    get_rotation_matrix(rotation_a_base_world, listener,
                                           "world", "ur5_arm_base_link");
    get_rotation_matrix(rotation_p_base_world, listener,
                                           "world", "base_link");
    twist_arm_world_frame = rotation_a_base_world * x_dot_a_
                           + rotation_p_base_world * x_dot_p_;
  }
}

bool AdmittanceController::get_rotation_matrix(Matrix6d & rotation_matrix,
                                                   tf::TransformListener & listener,
                                                   std::string from_frame,
                                                   std::string to_frame) {
  tf::StampedTransform transform;
  Matrix3d rotation_from_to;
  try{
    listener.lookupTransform(from_frame, to_frame,
                             ros::Time(0), transform);
    tf::matrixTFToEigen(transform.getBasis(), rotation_from_to);
    rotation_matrix.setZero();
    rotation_matrix.topLeftCorner(3, 3) = rotation_from_to;
    rotation_matrix.bottomRightCorner(3, 3) = rotation_from_to;
  }
  catch (tf::TransformException ex) {
    rotation_matrix.setZero();
    ROS_INFO("Waiting for TF...");
    return false;
  }

  return true;
}

// Wraps angle between -pi and pi
double AdmittanceController::wrap_angle(double angle) {
  if (angle>0) {
    return fmod(angle+M_PI, 2.0*M_PI)-M_PI;
  } else {
    return fmod(angle-M_PI, 2.0*M_PI)+M_PI;
  }
}
