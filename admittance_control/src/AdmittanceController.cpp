#include "AdmittanceController.h"

AdmittanceController::AdmittanceController(ros::NodeHandle &n,
    double frequency,
    std::string cmd_topic_platform,
    std::string state_topic_platform,
    std::string cmd_topic_arm,
    std::string topic_arm_pose_world,
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
    std::vector<double> workspace_limits,
    double wrench_filter_factor,
    double force_dead_zone_thres,
    double torque_dead_zone_thres,
    double obs_distance_thres,
    double self_detect_thres,
    bool dont_avoid_front) :
  nh_(n), loop_rate_(frequency),
  M_p_(M_p.data()), M_a_(M_a.data()), D_(D.data()),
  D_p_(D_p.data()), D_a_(D_a.data()), K_(K.data()),
  workspace_limits_(workspace_limits.data()),
  obs_distance_thres_(obs_distance_thres),
  self_detect_thres_(self_detect_thres),
  dont_avoid_front_(dont_avoid_front),
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
  pub_ee_pose_world_ = nh_.advertise<geometry_msgs::PoseStamped>(
                         topic_arm_pose_world, 5);
  pub_ee_twist_world_ = nh_.advertise<geometry_msgs::TwistStamped>(
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



  // initializing the class variables
  wrench_external_.setZero();
  wrench_control_.setZero();

  ee_pose_world_.setZero();
  ee_twist_world_.setZero();




  Vector7d equilibrium_full(d_e.data());

  equilibrium_position_ << equilibrium_full.topRows(3);

  // Make sure the orientation goal is normalized
  equilibrium_orientation_.coeffs() << equilibrium_full.bottomRows(4) /
                                    equilibrium_full.bottomRows(4).norm();


  // starting from a state that does not create movement on the robot
  arm_real_orientation_ = equilibrium_orientation_;

  // setting the robot state to zero and wait for data
  arm_real_position_.setZero();
  platform_real_position_.setZero();

  while (nh_.ok() && !arm_real_position_(0)) {
    ROS_WARN_THROTTLE(1, "Waiting for the state of the arm...");
    ros::spinOnce();
    loop_rate_.sleep();
  }
  ROS_INFO("Started to receive the state of the arm.");



  // ROS_INFO_STREAM("d_e_orientation :" << equilibrium_orientation_.coeffs());
  // ROS_INFO_STREAM("w : " << equilibrium_orientation_.w());
  // ROS_INFO_STREAM("vector : " << equilibrium_orientation_.vec());
  // ROS_INFO_STREAM("its norm : " << equilibrium_orientation_.norm() );





  // Kinematic constraints between base and arm at the equilibrium
  kin_constraints_.setZero();
  kin_constraints_.topLeftCorner(3, 3).setIdentity();
  kin_constraints_.bottomRightCorner(3, 3).setIdentity();
  // Screw on the z torque axis
  kin_constraints_.topRightCorner(3, 3) <<
                                        0, 0, equilibrium_position_(1),
                                        0, 0, -equilibrium_position_(0),
                                        0, 0, 0;
  obs_vector_.setZero();
  laser_front_cloud_.points.resize(0);
  laser_rear_cloud_.points.resize(0);


  ft_arm_ready_ = false;
  arm_world_ready_ = false;
  base_world_ready_ = false;
  world_arm_ready_ = false;

  init_TF();


}

////////////////////
/// CONTROL LOOP ///
////////////////////
void AdmittanceController::run() {

  ROS_INFO("Running the admittance control loop .................");

  // Init integrator
  arm_desired_twist_.setZero();
  platform_desired_twist_.setZero();


  while (nh_.ok()) {
    // Admittance Dynamics computation
    // compute_admittance(desired_twist_platform, desired_twist_arm,
    //                    loop_rate_.expectedCycleTime());
    compute_admittance();

    // limit the the movement of the arm to the permitted workspace
    limit_to_workspace();

    // Obstacle avoidance for the platform
    // avoid_obstacles(platform_desired_twist_);

    // Copy commands to messages
    send_commands_to_robot();

    // Arm pose/twist in the world frame
    // get_arm_twist_world(twist_arm_world_frame_, listener_arm_);
    publish_arm_state_in_world();

    // get_ee_pose_world(pose_ee_world_frame_, listener_arm_);

    // std::cout << "current pose of arm: " << pose_ee_world_frame_ << std::endl;

    // pub_ee_twist_world_.publish(twist_arm_world_frame_);

    // publishing visualization/debugging info
    publish_debuggings_signals();

    // For obstacle avoidance
    update_obstacles();

    ros::spinOnce();
    loop_rate_.sleep();
  }


}

// Admittance dynamics.
// void AdmittanceController::compute_admittance(Vector6d &desired_twist_platform,
//     Vector6d &desired_twist_arm,
//     ros::Duration duration)

void AdmittanceController::compute_admittance() {

  Vector6d platform_desired_acceleration;
  Vector6d arm_desired_accelaration;

  Vector6d error;

  // Translation error w.r.t. desired equilibrium
  error.topRows(3) = arm_real_position_ - equilibrium_position_;

  // Orientation error w.r.t. desired equilibriums
  if (equilibrium_orientation_.coeffs().dot(arm_real_orientation_.coeffs()) < 0.0) {
    arm_real_orientation_.coeffs() << -arm_real_orientation_.coeffs();
  }

  // ROS_INFO_STREAM("desired  :" << equilibrium_orientation_.coeffs());
  // ROS_INFO_STREAM("real :" << arm_real_orientation_.coeffs());



  Eigen::Quaterniond quat_rot_err(arm_real_orientation_
                                  * equilibrium_orientation_.inverse());
  if (quat_rot_err.coeffs().norm() > 1e-3) {
    // Normalize error quaternion
    quat_rot_err.coeffs() << quat_rot_err.coeffs() /
                          quat_rot_err.coeffs().norm();
  }
  Eigen::AngleAxisd err_arm_des_orient(quat_rot_err);
  error.bottomRows(3) << err_arm_des_orient.axis() *
                      err_arm_des_orient.angle();


  // ROS_INFO_STREAM("quat_rot_err :" << equilibrium_orientation_.coeffs());
  // ROS_INFO_STREAM("w : " << quat_rot_err.w());
  // ROS_INFO_STREAM("vector : " << quat_rot_err.vec());
  // ROS_INFO_STREAM("its norm : " << quat_rot_err.norm() );



  Vector6d coupling_wrench =  D_ * (arm_desired_twist_) + K_ * error;

  platform_desired_acceleration = M_p_.inverse() * (- D_p_ * platform_desired_twist_
                                  + rotation_base_ * kin_constraints_ * coupling_wrench);
  arm_desired_accelaration = M_a_.inverse() * ( - coupling_wrench - D_a_ * arm_desired_twist_
                             + wrench_external_ + wrench_control_);



  // limiting the accelaration for better stability and safety
  double p_acc_norm = (platform_desired_acceleration.segment(0, 3)).norm();
  double a_acc_norm = (arm_desired_accelaration.segment(0, 3)).norm();

  if (p_acc_norm > 2.0) {
    ROS_WARN_STREAM_THROTTLE(1, "Admittance generates high platform accelaration! accleration norm: " << p_acc_norm);
    platform_desired_acceleration.segment(0, 3) *= (2.0 / p_acc_norm);
  }

  if (a_acc_norm > 5.0) {
    ROS_WARN_STREAM_THROTTLE(1, "Admittance generates high arm accelaration! accleration norm: " << a_acc_norm);
    arm_desired_accelaration.segment(0, 3) *= (5.0 / a_acc_norm);
  }


  // Admittance dynamics
  // x_ddot_p = M_p_.inverse() * (- D_p_ * platform_desired_twist_
  //                              + rotation_base_ * kin_constraints_ *
  //                              (D_ * (arm_desired_twist_) + K_ * error));
  // x_ddot_a = M_a_.inverse() * ( - (D_ + D_a_) * (arm_desired_twist_)
  //                               - K_ * error + u_e_ + u_c_);

  // Integrate for velocity based interface
  ros::Duration duration = loop_rate_.expectedCycleTime();

  platform_desired_twist_ += platform_desired_acceleration * duration.toSec();
  arm_desired_twist_      += arm_desired_accelaration      * duration.toSec();

}

/////////////////
/// CALLBACKS ///
/////////////////
void AdmittanceController::state_platform_callback(
  const nav_msgs::OdometryConstPtr msg) {
  platform_real_position_ << msg->pose.pose.position.x, msg->pose.pose.position.y,
                          msg->pose.pose.position.z;
  platform_real_orientation_.coeffs() << msg->pose.pose.orientation.x,
                                    msg->pose.pose.orientation.y, msg->pose.pose.orientation.z,
                                    msg->pose.pose.orientation.w;

  platform_real_twist_ << msg->twist.twist.linear.x, msg->twist.twist.linear.y,
                       msg->twist.twist.linear.z, msg->twist.twist.angular.x,
                       msg->twist.twist.angular.y, msg->twist.twist.angular.z;
}

void AdmittanceController::state_arm_callback(
  const cartesian_state_msgs::PoseTwistConstPtr msg) {
  arm_real_position_ << msg->pose.position.x, msg->pose.position.y,
                     msg->pose.position.z;





  arm_real_orientation_.coeffs() << msg->pose.orientation.x,
                               msg->pose.orientation.y,
                               msg->pose.orientation.z,
                               msg->pose.orientation.w;

  // ROS_INFO_STREAM("Orientation of the arm (x-y-z-w) : " <<
  //                 msg->pose.orientation.x << " " <<
  //                 msg->pose.orientation.y << " " <<
  //                 msg->pose.orientation.z << " " <<
  //                 msg->pose.orientation.w  );

  arm_real_twist_ << msg->twist.linear.x, msg->twist.linear.y,
                  msg->twist.linear.z, msg->twist.angular.x, msg->twist.angular.y,
                  msg->twist.angular.z;



}

void AdmittanceController::wrench_callback(
  const geometry_msgs::WrenchStampedConstPtr msg) {
  Vector6d wrench_ft_frame;
  Matrix6d rotation_ft_base;
  if (ft_arm_ready_) {

    // Reading the FT-sensor in its own frame (robotiq_force_torque_frame_id)
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

    // Get transform from arm base link to platform base link
    get_rotation_matrix(rotation_ft_base, listener_ft_,
                        "ur5_arm_base_link", "robotiq_force_torque_frame_id");

    // Filter and update
    wrench_external_ <<  (1 - wrench_filter_factor_) * wrench_external_ +
                     wrench_filter_factor_ * rotation_ft_base * wrench_ft_frame;
  }
}

void AdmittanceController::wrench_control_callback(
  const geometry_msgs::WrenchStampedConstPtr msg) {
  /*Vector6d wrench_control_world_frame;
  Matrix6d rotation_world_base;
  if (world_arm_ready_) {
    get_rotation_matrix(rotation_world_base, listener_control_,
                        "ur5_arm_base_link", "world");
    wrench_control_world_frame << msg->wrench.force.x, msg->wrench.force.y,
                               msg->wrench.force.z,  msg->wrench.torque.x, msg->wrench.torque.y,
                               msg->wrench.torque.z;
    u_c_ << rotation_world_base * wrench_control_world_frame;
  }
  */
  if(msg->header.frame_id == "ur5_arm_base_link"){
      wrench_control_ << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z,
                  msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z;
  }
  else
  {
    ROS_WARN_THROTTLE(5,"The frame_id is not specified as ur5_arm_base_link");
  }



}


void AdmittanceController::send_commands_to_robot() {

  geometry_msgs::Twist platform_twist_cmd;
  geometry_msgs::Twist arm_twist_cmd;

  platform_twist_cmd.linear.x  = platform_desired_twist_(0);
  platform_twist_cmd.linear.y  = platform_desired_twist_(1);
  platform_twist_cmd.linear.z  = platform_desired_twist_(2);
  platform_twist_cmd.angular.x = platform_desired_twist_(3);
  platform_twist_cmd.angular.y = platform_desired_twist_(4);
  platform_twist_cmd.angular.z = platform_desired_twist_(5);

  arm_twist_cmd.linear.x  = arm_desired_twist_(0);
  arm_twist_cmd.linear.y  = arm_desired_twist_(1);
  arm_twist_cmd.linear.z  = arm_desired_twist_(2);
  arm_twist_cmd.angular.x = arm_desired_twist_(3);
  arm_twist_cmd.angular.y = arm_desired_twist_(4);
  arm_twist_cmd.angular.z = arm_desired_twist_(5);

  platform_pub_.publish(platform_twist_cmd);
  arm_pub_.publish(arm_twist_cmd);
}


void AdmittanceController::limit_to_workspace() {


  if (arm_real_position_(0) < workspace_limits_(0) || arm_real_position_(0) > workspace_limits_(1)) {
    ROS_WARN_STREAM_THROTTLE (1, "Out of permitted workspace.  x = "
                              << arm_real_position_(0) << " not in [" << workspace_limits_(0) << " , "
                              << workspace_limits_(1) << "]");
  }

  if (arm_real_position_(1) < workspace_limits_(2) || arm_real_position_(1) > workspace_limits_(3)) {
    ROS_WARN_STREAM_THROTTLE (1, "Out of permitted workspace.  y = "
                              << arm_real_position_(1) << " not in [" << workspace_limits_(2) << " , "
                              << workspace_limits_(3) << "]");
  }

  if (arm_real_position_(2) < workspace_limits_(4) || arm_real_position_(2) > workspace_limits_(5)) {
    ROS_WARN_STREAM_THROTTLE (1, "Out of permitted workspace.  z = "
                              << arm_real_position_(2) << " not in [" << workspace_limits_(4) << " , "
                              << workspace_limits_(5) << "]");
  }


  if (arm_desired_twist_(0) < 0 && arm_real_position_(0) < workspace_limits_(0)) {
    arm_desired_twist_(0) = 0;
  }

  if (arm_desired_twist_(0) > 0 && arm_real_position_(0) > workspace_limits_(1)) {
    arm_desired_twist_(0) = 0;
  }

  if (arm_desired_twist_(1) < 0 && arm_real_position_(1) < workspace_limits_(2)) {
    arm_desired_twist_(1) = 0;
  }

  if (arm_desired_twist_(1) > 0 && arm_real_position_(1) > workspace_limits_(3)) {
    arm_desired_twist_(1) = 0;
  }

  if (arm_desired_twist_(2) < 0 && arm_real_position_(2) < workspace_limits_(4)) {
    arm_desired_twist_(2) = 0;
  }

  if (arm_desired_twist_(2) > 0 && arm_real_position_(2) > workspace_limits_(5)) {
    arm_desired_twist_(2) = 0;
  }

  double norm_vel_des = (arm_desired_twist_.segment(0, 3)).norm();

  if (norm_vel_des > 1.0) {
    ROS_WARN_STREAM_THROTTLE(1, "Admittance generate fast movements! velocity norm: " << norm_vel_des);

    arm_desired_twist_.segment(0, 3) *= (1.0 / norm_vel_des);

  }
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
  ROS_INFO("The Force/Torque sensor is ready to use.");
}




////////////
/// UTIL ///
////////////

bool AdmittanceController::get_rotation_matrix(Matrix6d & rotation_matrix,
    tf::TransformListener & listener,
    std::string from_frame,
    std::string to_frame) {
  tf::StampedTransform transform;
  Matrix3d rotation_from_to;
  try {
    listener.lookupTransform(from_frame, to_frame,
                             ros::Time(0), transform);
    tf::matrixTFToEigen(transform.getBasis(), rotation_from_to);
    rotation_matrix.setZero();
    rotation_matrix.topLeftCorner(3, 3) = rotation_from_to;
    rotation_matrix.bottomRightCorner(3, 3) = rotation_from_to;
  }
  catch (tf::TransformException ex) {
    rotation_matrix.setZero();
    ROS_WARN_STREAM("Waiting for TF from: " << from_frame << " to: " << to_frame );
    return false;
  }

  return true;
}



void AdmittanceController::publish_arm_state_in_world() {
  // publishing the cartesian velocity of the EE in the world-frame
  Matrix6d rotation_a_base_world;
  Matrix6d rotation_p_base_world;


  if (arm_world_ready_ && base_world_ready_) {
    get_rotation_matrix(rotation_a_base_world, listener_arm_,
                        "world", "ur5_arm_base_link");
    get_rotation_matrix(rotation_p_base_world, listener_arm_,
                        "world", "base_link");

    ee_twist_world_ = rotation_a_base_world * arm_real_twist_
                      + rotation_p_base_world * platform_real_twist_;
    // ee_twist_world_ = arm_real_twist_ + platform_real_twist_;
  }

  geometry_msgs::TwistStamped msg_twist;
  msg_twist.header.stamp    = ros::Time::now();
  msg_twist.header.frame_id = "world";
  msg_twist.twist.linear.x = ee_twist_world_(0);
  msg_twist.twist.linear.y = ee_twist_world_(1);
  msg_twist.twist.linear.z = ee_twist_world_(2);
  msg_twist.twist.angular.x = ee_twist_world_(3);
  msg_twist.twist.angular.y = ee_twist_world_(4);
  msg_twist.twist.angular.z = ee_twist_world_(5);
  pub_ee_twist_world_.publish(msg_twist);


  // publishing the cartesian position of the EE in the world-frame

  tf::StampedTransform transform;


  if (arm_world_ready_ && base_world_ready_) {
    try {
      // listener.lookupTransform("ur5_arm_base_link", "robotiq_force_torque_frame_id",
      listener_arm_.lookupTransform("world", "robotiq_force_torque_frame_id",
                                    ros::Time(0), transform);

      // transform.getRotation().getW();

      ee_pose_world_(0) = transform.getOrigin().x();
      ee_pose_world_(1) = transform.getOrigin().y();
      ee_pose_world_(2) = transform.getOrigin().z();
      ee_pose_world_(3) = transform.getRotation().x();
      ee_pose_world_(4) = transform.getRotation().y();
      ee_pose_world_(5) = transform.getRotation().z();
      ee_pose_world_(6) = transform.getRotation().w();
    }
    catch (tf::TransformException ex) {
      ROS_WARN("Couldn't lookup for ee to world transform...");
      ee_pose_world_.setZero();
      ee_pose_world_(6) = 1; // quat.w = 1
    }
  }

  geometry_msgs::PoseStamped msg_pose;
  msg_pose.header.stamp    = ros::Time::now();
  msg_pose.header.frame_id = "world";
  msg_pose.pose.position.x = ee_pose_world_(0);
  msg_pose.pose.position.y = ee_pose_world_(1);
  msg_pose.pose.position.z = ee_pose_world_(2);
  msg_pose.pose.orientation.x = ee_pose_world_(3);
  msg_pose.pose.orientation.y = ee_pose_world_(4);
  msg_pose.pose.orientation.z = ee_pose_world_(5);
  msg_pose.pose.orientation.w = ee_pose_world_(6);
  pub_ee_pose_world_.publish(msg_pose);

}



void AdmittanceController::publish_debuggings_signals() {

  geometry_msgs::WrenchStamped wrench_msg;

  wrench_msg.header.stamp    = ros::Time::now();
  wrench_msg.header.frame_id = "ur5_arm_base_link";
  wrench_msg.wrench.force.x  = wrench_external_(0);
  wrench_msg.wrench.force.y  = wrench_external_(1);
  wrench_msg.wrench.force.z  = wrench_external_(2);
  wrench_msg.wrench.torque.x = wrench_external_(3);
  wrench_msg.wrench.torque.y = wrench_external_(4);
  wrench_msg.wrench.torque.z = wrench_external_(5);
  wrench_pub_u_e_.publish(wrench_msg);

  wrench_msg.header.stamp    = ros::Time::now();
  wrench_msg.header.frame_id = "ur5_arm_base_link";
  wrench_msg.wrench.force.x  = wrench_control_(0);
  wrench_msg.wrench.force.y  = wrench_control_(1);
  wrench_msg.wrench.force.z  = wrench_control_(2);
  wrench_msg.wrench.torque.x = wrench_control_(3);
  wrench_msg.wrench.torque.y = wrench_control_(4);
  wrench_msg.wrench.torque.z = wrench_control_(5);
  wrench_pub_u_c_.publish(wrench_msg);


  geometry_msgs::PointStamped obs_pub_msg;

  obs_pub_msg.header.stamp = ros::Time::now();
  obs_pub_msg.header.frame_id = "base_link";
  obs_pub_msg.point.x = obs_vector_(0);
  obs_pub_msg.point.y = obs_vector_(1);
  obs_pub_msg.point.z = obs_vector_(2);
  obs_pub_.publish(obs_pub_msg);

}













//////////////////////////////////////////////////////////////////
/// Obstacale avoidance which need to be moved to another node ///
//////////////////////////////////////////////////////////////////

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

  for (unsigned int i = 0 ; i < laser_front_cloud_.points.size() ; i ++) {
    Eigen::Vector3d cur_vector_front;
    cur_vector_front << laser_front_cloud_.points.at(i).x,
                     laser_front_cloud_.points.at(i).y,
                     laser_front_cloud_.points.at(i).z;

    if (isObstacleMeasurement(cur_vector_front)) {
      sum_vectors = sum_vectors + cur_vector_front;
      n_vectors++;
    }
  }
  for (unsigned int i = 0 ; i < laser_rear_cloud_.points.size() ; i ++) {
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
    obs_vector_ = sum_vectors / n_vectors;
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
          std::abs(measurement(1)) > (0.4 + self_detect_thres_)) &&
         (!dont_avoid_front_ || measurement(0) < 0.48);
}