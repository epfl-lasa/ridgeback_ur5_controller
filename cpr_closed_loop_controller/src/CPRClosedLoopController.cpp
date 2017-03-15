#include "CPRClosedLoopController.h"
namespace CPR {
CPRClosedLoopController::CPRClosedLoopController(ros::NodeHandle &n,
                                                 double frequency,
                                                 std::string cmd_topic_platform,
                                                 std::string odm_topic_platform,
                                                 std::string host_arm,
                                                 int reverse_port_arm) :
                                                 nh_(n),
                                                 loop_rate_(frequency),
                                                 ur5_arm_(rt_msg_cond_,
                                                          msg_cond_,
                                                          host_arm,
                                                          reverse_port_arm,
                                                          0.03,
                                                          300) {
  rb_twist_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_topic_platform, 1);
  odom_sub_ = nh_.subscribe(odm_topic_platform, 5,
                                &CPRClosedLoopController::odom_callback, this,
                                ros::TransportHints().reliable().tcpNoDelay());
}


void CPRClosedLoopController::run() {
  // Desired twists
  Vector6d desired_twist_arm;
  Vector6d desired_twist_platform;

  // A placeholder to initialize whatever is needed for the computeVelCmd right
  // before starting the loop
  init();

  while(nh_.ok()) {
    updateRobotState();

    // A placeholder to the main method of the controller
    computeVelCmd(desired_twist_platform, desired_twist_arm);

    // Copy commands to messages
    platform_twist_.linear.x = desired_twist_platform(0);
    platform_twist_.linear.y = desired_twist_platform(1);
    platform_twist_.linear.z = desired_twist_platform(2);
    platform_twist_.angular.x = desired_twist_platform(3);
    platform_twist_.angular.y = desired_twist_platform(4);
    platform_twist_.angular.z = desired_twist_platform(5);

    rb_twist_pub_.publish(platform_twist_);

    ros::spinOnce();

    loop_rate_.sleep();
  }
}

// Read the state of the arm and the platform and copy it to the local
// attributes
void CPRClosedLoopController::updateRobotState() {

  // Read UR arm state
  std::vector<double> buf;
  buf.resize(6);
  Map<const Vector6d> ret(buf.data());
  buf = ur5_arm_.rt_interface_->robot_state_->getQActual();
  joint_state_arm_ << ret;
  //std::cout << ur5_arm_.rt_interface_->connected_;
  buf = ur5_arm_.rt_interface_->robot_state_->getQdActual();
  joint_state_vel_arm_ << ret;
  //std::cout << buf.at(0);
  buf = ur5_arm_.rt_interface_->robot_state_->getToolVectorActual();
  cart_state_arm_ << ret;
  //std::cout << buf.at(1);
  buf = ur5_arm_.rt_interface_->robot_state_->getTcpSpeedActual();
  cart_state_twist_arm_ << ret;
  //std::cout << buf.at(0);
  buf = ur5_arm_.rt_interface_->robot_state_->getTcpForce();
  //ft_cart_ << ret;
  buf = ur5_arm_.rt_interface_->robot_state_->getToolAccelerometerValues();
  //accel_ee_ << ret;

  // Platform state is read from the odom subscriber
}

void CPRClosedLoopController::odom_callback(
    const geometry_msgs::PoseStamped::ConstPtr& msg) {
  cart_state_platform_ << msg->pose.position.x, msg->pose.position.y,
      msg->pose.position.z, msg->pose.orientation.x, msg->pose.orientation.y,
      msg->pose.orientation.z, msg->pose.orientation.w;
}

// A placeholder to initialize whatever is needed for the computeVelCmd right
// before starting the loop
void CPRClosedLoopController::init() {}

// A placeholder to the main method of the controller
void CPRClosedLoopController::computeVelCmd(Vector6d & desired_twist_platform,
                                            Vector6d & desired_joint_vel_arm) {
  desired_twist_platform.setZero();
  desired_twist_platform(1) = 1;
  desired_joint_vel_arm.setZero();
}

}
