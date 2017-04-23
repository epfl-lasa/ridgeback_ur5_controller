#include "AdmittanceController.h"

AdmittanceController::AdmittanceController(ros::NodeHandle &n,
                                              double frequency,
                                              std::string cmd_topic_platform,
                                              std::string state_topic_platform,
                                              std::string cmd_topic_arm,
                                              std::string state_topic_arm,
                                              std::string wrench_topic,
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
  platform_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_topic_platform, 1);
  platform_sub_ = nh_.subscribe(state_topic_platform, 1,
                          &AdmittanceController::state_platform_callback, this,
                          ros::TransportHints().reliable().tcpNoDelay());
  wrench_sub_ = nh_.subscribe(wrench_topic, 1,
                          &AdmittanceController::wrench_callback, this,
                          ros::TransportHints().reliable().tcpNoDelay());
  arm_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_topic_arm, 1);
  arm_sub_ = nh_.subscribe(state_topic_arm, 1,
                          &AdmittanceController::state_arm_callback, this,
                          ros::TransportHints().reliable().tcpNoDelay());

  // Get transform from arm base link to platform base link
  tf::TransformListener listener;
  tf::StampedTransform transform;
  Eigen::Matrix3d rotation_base;
  try{
    listener.waitForTransform("/ur5_arm_base_link","/base_link",
                              ros::Time(0), ros::Duration(10.0) );
    listener.lookupTransform( "/ur5_arm_base_link","/base_link",
              ros::Time(0), transform);
  }
  catch (tf::TransformException ex) {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
  tf::matrixTFToEigen(transform.getBasis().inverse(), rotation_base);

  rotation_base_.setZero();
  rotation_base_.topLeftCorner(3, 3) = rotation_base;
  rotation_base_.bottomRightCorner(3, 3) = rotation_base;
}

// Control loop
void AdmittanceController::run() {
  // Desired twists
  Vector6d desired_twist_arm;
  Vector6d desired_twist_platform;
  geometry_msgs::Twist platform_twist_cmd;
  geometry_msgs::Twist arm_twist_cmd;

  while(nh_.ok()) {
    // Dynamics computation
    compute_admittance(desired_twist_platform, desired_twist_arm,
                       loop_rate_.expectedCycleTime());

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

    platform_pub_.publish(platform_twist_cmd);
    arm_pub_.publish(arm_twist_cmd);

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
                             - (D_a_ * x_dot_a_) + K_ * (d_e_ - x_a_) + u_e_);

  // Integrate for velocity based interface
  desired_twist_platform = x_dot_p_ + x_ddot_p * duration.toSec();
  desired_twist_arm = x_dot_a_ + x_ddot_a * duration.toSec();

  std::cout << "Desired twist arm: " << desired_twist_arm << std::endl;
  std::cout << "Desired twist platform: " << desired_twist_platform << std::endl;
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
  u_e_ << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z,
          msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z;
}
