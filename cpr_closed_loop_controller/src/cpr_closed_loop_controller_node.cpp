#include "ros/ros.h"
#include "CPRClosedLoopController.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cpr_closed_loop_controller_node");

  ros::NodeHandle nh;
  double frequency = 300.0;

  // Parameters
  int reverse_port_arm;
  std::string ip_address_arm, cmd_topic_platform, odom_topic_platform;

  if (!nh.getParam("ip_address_arm", ip_address_arm))
  {
    ROS_ERROR("Couldn't retrieve the arm_ip_address. ");
    return -1;
  }

  if (!nh.getParam("cmd_topic_platform", cmd_topic_platform))
  {
    ROS_ERROR("Couldn't retrieve the cmd_topic_platform. ");
    return -1;
  }

  if (!nh.getParam("odom_topic_platform", odom_topic_platform))
  {
    ROS_ERROR("Couldn't retrieve the odom_topic_platform. ");
    return -1;
  }

  if (!nh.getParam("reverse_port_arm", reverse_port_arm))
  {
    ROS_ERROR("Couldn't retrieve the number_of_slaves. ");
    return -1;
  }

  CPR::CPRClosedLoopController cpr_closed_loop_controller(nh, frequency,
                                                    cmd_topic_platform,
                                                    odom_topic_platform,
                                                    ip_address_arm,
                                                    reverse_port_arm);
  cpr_closed_loop_controller.run();

  return 0;
}
