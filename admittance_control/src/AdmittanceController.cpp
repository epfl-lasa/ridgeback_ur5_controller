#include "AdmittanceController.h"

AdmittanceController::AdmittanceController(ros::NodeHandle &n,
                                              double frequency,
                                              std::string cmd_topic_platform,
                                              std::string state_topic_platform,
                                              std::string cmd_topic_arm,
                                              std::string state_topic_arm,
                                              std::string wrench_topic) :
                                              nh_(n),
                                              loop_rate_(frequency) {
  platform_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_topic_platform, 1);
  platform_sub_ = nh_.subscribe(state_topic_platform, 5,
                          &AdmittanceController::state_platform_callback, this,
                          ros::TransportHints().reliable().tcpNoDelay());
  wrench_sub_ = nh_.subscribe(wrench_topic, 5,
                          &AdmittanceController::wrench_callback, this,
                          ros::TransportHints().reliable().tcpNoDelay());
  arm_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_topic_arm, 1);
  arm_sub_ = nh_.subscribe(state_topic_arm, 5,
                          &AdmittanceController::state_arm_callback, this,
                          ros::TransportHints().reliable().tcpNoDelay());
}


void AdmittanceController::run() {
  // Desired twists
  Vector6d desired_twist_arm;
  Vector6d desired_twist_platform;
  geometry_msgs::Twist platform_twist_cmd;
  geometry_msgs::Twist arm_twist_cmd;

  while(nh_.ok()) {
    // Dynamics computation
    compute_admittance(desired_twist_platform, desired_twist_arm,
                       loop_rate_.cycleTime());

    // Copy commands to messages
    platform_twist_cmd.linear.x = desired_twist_platform(0);
    platform_twist_cmd.linear.y = desired_twist_platform(1);
    platform_twist_cmd.linear.z = desired_twist_platform(2);
    platform_twist_cmd.angular.x = desired_twist_platform(3);
    platform_twist_cmd.angular.y = desired_twist_platform(4);
    platform_twist_cmd.angular.z = desired_twist_platform(5);

    arm_twist_cmd.linear.x = desired_twist_platform(0);
    arm_twist_cmd.linear.y = desired_twist_platform(1);
    arm_twist_cmd.linear.z = desired_twist_platform(2);
    arm_twist_cmd.angular.x = desired_twist_platform(3);
    arm_twist_cmd.angular.y = desired_twist_platform(4);
    arm_twist_cmd.angular.z = desired_twist_platform(5);

    platform_pub_.publish(platform_twist_cmd);
    arm_pub_.publish(platform_twist_cmd);

    ros::spinOnce();

    loop_rate_.sleep();
  }
}

void AdmittanceController::state_platform_callback(
    const nav_msgs::OdometryConstPtr msg) {
    // copy to Eigen variables
  cart_state_platform_ << msg->pose.pose.position.x, msg->pose.pose.position.y,
    msg->pose.pose.position.z, msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y, msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w;

  cart_twist_platform_ << msg->twist.twist.linear.x, msg->twist.twist.linear.y,
    msg->twist.twist.linear.z, msg->twist.twist.angular.x,
    msg->twist.twist.angular.y, msg->twist.twist.angular.z;
}

void AdmittanceController::state_arm_callback(
    const cartesian_state_msgs::PoseTwistConstPtr msg) {
  // copy to Eigen variables
  cart_state_arm_ << msg->pose.position.x, msg->pose.position.y,
          msg->pose.position.z, msg->pose.orientation.x,
          msg->pose.orientation.y, msg->pose.orientation.z,
          msg->pose.orientation.w;
  cart_twist_arm_ << msg->twist.linear.x, msg->twist.linear.y,
          msg->twist.linear.z, msg->twist.angular.x, msg->twist.angular.y,
          msg->twist.angular.z;
}

void AdmittanceController::wrench_callback(
    const geometry_msgs::WrenchConstPtr msg) {
  // copy to Eigen variables
  cart_ft_ << msg->force.x, msg->force.y, msg->force.z, msg->torque.x,
    msg->torque.y, msg->torque.z;
}

// Admittance dynamics.
void AdmittanceController::compute_admittance(Vector6d &desired_twist_platform,
                                            Vector6d &desired_twist_arm,
                                            ros::Duration duration) {
  desired_twist_platform.setZero();
  desired_twist_platform(0) = 1;
  desired_twist_arm.setZero();
}


