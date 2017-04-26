#include "AdmittanceController.h"

AdmittanceController::AdmittanceController(ros::NodeHandle &n,
                                              double frequency,
                                              std::string cmd_topic_platform,
                                              std::string state_topic_platform,
                                              std::string cmd_topic_arm,
                                              std::string topic_arm_twist_world,
                                              std::string state_topic_arm,
                                              std::string wrench_topic,
                                              std::string wrench_control_topic,
                                              std::vector<double> M_p,
                                              std::vector<double> M_a,
                                              std::vector<double> D,
                                              std::vector<double> D_p,
                                              std::vector<double> D_a,
                                              std::vector<double> K,
                                              std::vector<double> d_e) :
                                              nh_(n),
                                              loop_rate_(frequency),
                                              M_p_(M_p.data()),
                                              M_a_(M_a.data()), D_(D.data()),
                                              D_p_(D_p.data()),
                                              D_a_(D_a.data()), K_(K.data()),
                                              d_e_(d_e.data()) {
  platform_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_topic_platform, 3);
  platform_sub_ = nh_.subscribe(state_topic_platform, 3,
                          &AdmittanceController::state_platform_callback, this,
                          ros::TransportHints().reliable().tcpNoDelay());
  wrench_sub_ = nh_.subscribe(wrench_topic, 3,
                          &AdmittanceController::wrench_callback, this,
                          ros::TransportHints().reliable().tcpNoDelay());
  wrench_control_sub_ = nh_.subscribe(wrench_control_topic, 3,
                          &AdmittanceController::wrench_control_callback, this,
                          ros::TransportHints().reliable().tcpNoDelay());
  arm_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_topic_arm, 3);
  arm_pub_world_ = nh_.advertise<geometry_msgs::Twist>(
                                                    topic_arm_twist_world, 3);
  arm_sub_ = nh_.subscribe(state_topic_arm, 3,
                          &AdmittanceController::state_arm_callback, this,
                          ros::TransportHints().reliable().tcpNoDelay());

  tf::TransformListener listener;
  // Get transform from arm base link to platform base link
  while (!get_rotation_matrix(rotation_base_, listener,
                                     "ur5_arm_base_link", "base_link")) {
    sleep(0.1);
  }

  u_e_.setZero();
  u_c_.setZero();
}

// Control loop
void AdmittanceController::run() {
  // Desired twists
  Vector6d desired_twist_arm;
  Vector6d desired_twist_platform;
  Vector6d twist_arm_world_frame;
  geometry_msgs::Twist platform_twist_cmd;
  geometry_msgs::Twist arm_twist_cmd;
  geometry_msgs::Twist arm_twist_world;
  tf::TransformListener listener;

  while(nh_.ok()) {
    // Dynamics computation
    compute_admittance(desired_twist_platform, desired_twist_arm,
                       loop_rate_.expectedCycleTime());
    // Arm twist in the world frame
    get_arm_twist_world(twist_arm_world_frame, listener);

    // Copy commands to messages
    platform_twist_cmd.linear.x = desired_twist_platform(0);
    platform_twist_cmd.linear.y = desired_twist_platform(1);
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

    arm_twist_world.linear.x  = twist_arm_world_frame(0);
    arm_twist_world.linear.y  = twist_arm_world_frame(1);
    arm_twist_world.linear.z  = twist_arm_world_frame(2);
    arm_twist_world.angular.x = twist_arm_world_frame(3);
    arm_twist_world.angular.y = twist_arm_world_frame(4);
    arm_twist_world.angular.z = twist_arm_world_frame(5);

    platform_pub_.publish(platform_twist_cmd);
    arm_pub_.publish(arm_twist_cmd);
    arm_pub_world_.publish(arm_twist_world);

    ros::spinOnce();

    loop_rate_.sleep();
  }
}

// Admittance dynamics.
void AdmittanceController::compute_admittance(Vector6d &desired_twist_platform,
                                            Vector6d &desired_twist_arm,
                                            ros::Duration duration) {
  Vector6d x_ddot_p, x_ddot_a;
  x_ddot_p = M_p_.inverse()*(- D_*(rotation_base_* x_dot_a_)
                 - (D_p_ * x_dot_p_) - K_  * (rotation_base_* (d_e_ - x_a_)) );
  x_ddot_a = M_a_.inverse()*( - D_*(x_dot_a_)
                             - (D_a_ * x_dot_a_) + K_ * (d_e_ - x_a_) + u_e_ + u_c_);

  // Integrate for velocity based interface
  desired_twist_platform = x_dot_p_ + x_ddot_p * duration.toSec();
  desired_twist_arm = x_dot_a_ + x_ddot_a * duration.toSec();

  std::cout << "Desired twist arm: " << desired_twist_arm << std::endl;
  std::cout << "Desired twist platform: " << desired_twist_platform << std::endl;
}

void AdmittanceController::get_arm_twist_world(Vector6d &twist_arm_world_frame,
                                            tf::TransformListener & listener) {
  // publishing the cartesian velocity of the EE in the world-frame
  Matrix6d rotation_a_base_world;
  Matrix6d rotation_p_base_world;

  get_rotation_matrix(rotation_a_base_world, listener,
                                           "ur5_arm_base_link","world");
  get_rotation_matrix(rotation_p_base_world, listener,
                                           "base_link","world");
  twist_arm_world_frame = rotation_a_base_world * x_dot_a_
                           + rotation_p_base_world * x_dot_p_;
}

// CALLBACKS
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

  x_dot_a_ << msg->twist.linear.x, msg->twist.linear.y,
          msg->twist.linear.z, msg->twist.angular.x, msg->twist.angular.y,
          msg->twist.angular.z;
}

void AdmittanceController::wrench_callback(
    const geometry_msgs::WrenchStampedConstPtr msg) {
    // Get transform from arm base link to platform base link
  Vector6d wrench_ft_frame;
  Matrix6d rotation_ft_base;
  get_rotation_matrix(rotation_ft_base, listener_ft_,
                                         "FT300_link", "ur5_arm_base_link");

  wrench_ft_frame << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z,
          msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z;
  u_e_ << rotation_ft_base * wrench_ft_frame;
}

void AdmittanceController::wrench_control_callback(
    const geometry_msgs::WrenchStampedConstPtr msg) {
  Vector6d wrench_control_world_frame;
  Matrix6d rotation_world_base;
  get_rotation_matrix(rotation_world_base, listener_control_,
                                             "world", "ur5_arm_base_link");
  wrench_control_world_frame << msg->wrench.force.x, msg->wrench.force.y,
          msg->wrench.force.z,  msg->wrench.torque.x, msg->wrench.torque.y,
          msg->wrench.torque.z;
  u_c_ << rotation_world_base * wrench_control_world_frame;
}

bool AdmittanceController::get_rotation_matrix(Matrix6d & rotation_matrix,
                                                   tf::TransformListener & listener,
                                                   std::string from_frame,
                                                   std::string to_frame) {
  tf::StampedTransform transform;
  Matrix3d rotation_from_to;
  ros::Time now = ros::Time::now();
  try{
    listener.waitForTransform(from_frame, to_frame,
                              now, ros::Duration(0.1) );
    listener.lookupTransform(from_frame, to_frame,
                             now, transform);
    tf::matrixTFToEigen(transform.getBasis().inverse(), rotation_from_to);
    rotation_matrix.setZero();
    rotation_matrix.topLeftCorner(3, 3) = rotation_from_to;
    rotation_matrix.bottomRightCorner(3, 3) = rotation_from_to;
  }
  catch (tf::TransformException ex) {
    ROS_WARN("%s",ex.what());
    rotation_matrix.setZero();
    return false;
  }

  return true;
}


