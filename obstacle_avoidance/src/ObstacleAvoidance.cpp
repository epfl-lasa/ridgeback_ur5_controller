#include "ObstacleAvoidance.h"

ObstacleAvoidance::ObstacleAvoidance(ros::NodeHandle &n,
                                     double frequency,
                                     std::string topic_platform_state,
                                     std::string topic_platform_desired_twist,
                                     std::string topic_platform_command,
                                     std::string topic_laser_front,
                                     std::string topic_laser_rear,
                                     double obs_distance_thres,
                                     double self_detect_thres,
                                     bool dont_avoid_front) :
  nh_(n), loop_rate_(frequency),
  obs_distance_thres_(obs_distance_thres),
  self_detect_thres_(self_detect_thres),
  dont_avoid_front_(dont_avoid_front) {


  // Subscribers
  sub_platform_state_ = nh_.subscribe(topic_platform_state, 5,
                                      &ObstacleAvoidance::state_platform_callback, this,
                                      ros::TransportHints().reliable().tcpNoDelay());

  sub_platform_desired_twist_ = nh_.subscribe(topic_platform_desired_twist, 1,
                                &ObstacleAvoidance::desired_velocity_callback, this,
                                ros::TransportHints().reliable().tcpNoDelay());


  sub_laser_front_ = nh_.subscribe(topic_laser_front, 1,
                                   &ObstacleAvoidance::laser_front_callback, this,
                                   ros::TransportHints().reliable().tcpNoDelay());

  sub_laser_rear_ = nh_.subscribe(topic_laser_rear, 1,
                                  &ObstacleAvoidance::laser_rear_callback, this,
                                  ros::TransportHints().reliable().tcpNoDelay());

  // Publishers
  pub_platform_cmd_ = nh_.advertise<geometry_msgs::Twist>(topic_platform_command, 5);

  pub_obstacle_position_ = nh_.advertise<geometry_msgs::PointStamped>("obstacles", 5);


  // initializing the class variables
  platform_desired_twist_.setZero();
  platform_desired_twist_modified_.setZero();

  obs_vector_.setZero();
  laser_front_cloud_.points.resize(0);
  laser_rear_cloud_.points.resize(0);


  base_world_ready_ = false;

  wait_for_transformations();
}

///////////////////////////////////////////////////////////////
///////////////////// Control Loop ////////////////////////////
///////////////////////////////////////////////////////////////
void ObstacleAvoidance::run() {

  ROS_INFO("Running the Obstacle Avoidance loop .................");


  while (nh_.ok()) {

    // Obstacle avoidance for the platform
    update_obstacles();
    avoid_obstacles();

    // Command the robot
    send_commands_to_platform();

    ros::spinOnce();
    loop_rate_.sleep();
  }

}

//////////////////////////////////////////////////////////////////
/////////////////////// Obstacale avoidance //////////////////////
//////////////////////////////////////////////////////////////////
void ObstacleAvoidance::avoid_obstacles() {

  platform_desired_twist_modified_ = platform_desired_twist_;

  // Assumption: if obs_vector = (0,0,0) there is no obstacle
  double hard_threshold = obs_distance_thres_ / 2.0;
  if (obs_vector_.norm() > 0.01) {
    Vector3d closest_point = get_closest_point_on_platform(obs_vector_);
    Vector3d obs_closest_point_frame = obs_vector_ - closest_point;
    obs_closest_point_frame(2) = 0.0;
    // If there is a component of the velocity driving the platform to the
    // obstacle ...
    if (platform_desired_twist_.topRows(3).dot(obs_closest_point_frame) > 0.0) {
      // ... remove it lineary until fully removed at hard_threshold
      double factor = (1.0 - ((obs_closest_point_frame.norm()
                               - (obs_distance_thres_ - hard_threshold))
                              / hard_threshold));
      factor = std::min(std::max(factor, 0.0), 1.0);
      platform_desired_twist_modified_.topRows(3) = platform_desired_twist_.topRows(3)
          - factor *
          (platform_desired_twist_.topRows(3).dot(obs_closest_point_frame)
           * obs_closest_point_frame / obs_closest_point_frame.squaredNorm());
    }
  }
}

///////////////////////////////////////////////////////////////
//////////////////// COMMANDING THE ROBOT /////////////////////
///////////////////////////////////////////////////////////////
void ObstacleAvoidance::send_commands_to_platform() {

  geometry_msgs::Twist platform_twist_cmd;

  platform_twist_cmd.linear.x  = platform_desired_twist_modified_(0);
  platform_twist_cmd.linear.y  = platform_desired_twist_modified_(1);
  platform_twist_cmd.linear.z  = platform_desired_twist_modified_(2);
  platform_twist_cmd.angular.x = platform_desired_twist_modified_(3);
  platform_twist_cmd.angular.y = platform_desired_twist_modified_(4);
  platform_twist_cmd.angular.z = platform_desired_twist_modified_(5);

  pub_platform_cmd_.publish(platform_twist_cmd);
}

///////////////////////////////////////////////////////////////
////////////////////////// Callbacks //////////////////////////
///////////////////////////////////////////////////////////////
void ObstacleAvoidance::state_platform_callback(
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

void ObstacleAvoidance::desired_velocity_callback(
  const geometry_msgs::TwistConstPtr msg) {
  platform_desired_twist_ << msg->linear.x, msg->linear.y, msg->linear.z,
                          msg->angular.x, msg->angular.y, msg->angular.z;


  ROS_INFO_STREAM_THROTTLE(1, "receiving desired twist " << platform_desired_twist_);
}

void ObstacleAvoidance::laser_front_callback(
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

void ObstacleAvoidance::laser_rear_callback(
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




//////////////////////
/// INITIALIZATION ///
//////////////////////
void ObstacleAvoidance::wait_for_transformations() {
  tf::TransformListener listener;
  Matrix6d rot_matrix;
  rotation_base_.setZero();

  // Makes sure world-base transform exists
  while (!get_rotation_matrix(rot_matrix, listener,
                              "world", "base_link")) {
    sleep(1);
  }
  base_world_ready_ = true;

}




////////////
/// UTIL ///
////////////
bool ObstacleAvoidance::get_rotation_matrix(Matrix6d & rotation_matrix,
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

Vector3d ObstacleAvoidance::get_closest_point_on_platform(Vector3d obstacle) {
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

void ObstacleAvoidance::update_obstacles() {
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

  geometry_msgs::PointStamped msg_point;
  msg_point.header.frame_id = "base_link";
  msg_point.header.stamp = ros::Time::now();
  msg_point.point.x = obs_vector_(0);
  msg_point.point.y = obs_vector_(1);
  msg_point.point.z = obs_vector_(2);

  pub_obstacle_position_.publish(msg_point);




}

bool ObstacleAvoidance::isObstacleMeasurement(Vector3d &measurement) {
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