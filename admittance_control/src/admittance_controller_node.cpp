#include "ros/ros.h"
#include "AdmittanceController.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "admittance_controller_node");

  ros::NodeHandle nh;
  double frequency = 300.0;

  // Parameters
  std::string state_topic_arm, cmd_topic_arm,
          cmd_topic_platform, state_topic_platform, wrench_topic;

  if (!nh.getParam("state_topic_arm", state_topic_arm))
  {
    ROS_ERROR("Couldn't retrieve the state_topic_arm. ");
    return -1;
  }

  if (!nh.getParam("cmd_topic_platform", cmd_topic_platform))
  {
    ROS_ERROR("Couldn't retrieve the cmd_topic_platform. ");
    return -1;
  }

  if (!nh.getParam("state_topic_platform", state_topic_platform))
  {
    ROS_ERROR("Couldn't retrieve the state_topic_platform. ");
    return -1;
  }

  if (!nh.getParam("cmd_topic_arm", cmd_topic_arm))
  {
    ROS_ERROR("Couldn't retrieve the cmd_topic_arm. ");
    return -1;
  }

  if (!nh.getParam("wrench_topic", wrench_topic))
  {
    ROS_ERROR("Couldn't retrieve the cmd_topic_arm. ");
    return -1;
  }

  AdmittanceController admittance_controller(nh, frequency,
                                             cmd_topic_platform,
                                             state_topic_platform,
                                             cmd_topic_arm,
                                             state_topic_arm,
                                             wrench_topic);
  admittance_controller.run();

  return 0;
}

