#include "ros/ros.h"
#include "AdmittanceController.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "admittance_controller_node");

  ros::NodeHandle nh;
  double frequency = 125.0;

  // Parameters
  std::string state_topic_arm, cmd_topic_arm, topic_arm_twist_world, topic_wrench_u_e, topic_wrench_u_c,
          cmd_topic_platform, state_topic_platform, wrench_topic,
	  wrench_control_topic;
  std::vector<double> M_p, M_a, D, D_p, D_a, K, d_e;

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

  if (!nh.getParam("topic_arm_twist_world", topic_arm_twist_world))
  {
    ROS_ERROR("Couldn't retrieve the topic_arm_twist_world. ");
    return -1;
  }

  if (!nh.getParam("topic_wrench_u_e", topic_wrench_u_e))
  {
    ROS_ERROR("Couldn't retrieve the topic_wrench_u_e. ");
    return -1;
  }

  if (!nh.getParam("topic_wrench_u_c", topic_wrench_u_c))
  {
    ROS_ERROR("Couldn't retrieve the topic_wrench_u_c. ");
    return -1;
  }

  if (!nh.getParam("wrench_topic", wrench_topic))
  {
    ROS_ERROR("Couldn't retrieve the wrench_topic. ");
    return -1;
  }

  if (!nh.getParam("wrench_control_topic", wrench_control_topic))
  {
    ROS_ERROR("Couldn't retrieve the wrench_control_topic. ");
    return -1;
  }

  // ADMITTANCE PARAMS
  if (!nh.getParam("mass_platform", M_p))
  {
    ROS_ERROR("Couldn't retrieve the desired mass platform. ");
    return -1;
  }

  if (!nh.getParam("mass_arm", M_a))
  {
    ROS_ERROR("Couldn't retrieve the desired mass of the arm. ");
    return -1;
  }

  if (!nh.getParam("damping_coupling", D))
  {
    ROS_ERROR("Couldn't retrieve the desired damping of the coupling. ");
    return -1;
  }

  if (!nh.getParam("damping_platform", D_p))
  {
    ROS_ERROR("Couldn't retrieve the desired damping of the platform. ");
    return -1;
  }

  if (!nh.getParam("damping_arm", D_a))
  {
    ROS_ERROR("Couldn't retrieve the desired damping of the arm. ");
    return -1;
  }

  if (!nh.getParam("stiffness_coupling", K))
  {
    ROS_ERROR("Couldn't retrieve the desired stiffness of the coupling. ");
    return -1;
  }

  if (!nh.getParam("equilibrium_point_spring", d_e))
  {
    ROS_ERROR("Couldn't retrieve the desired equilibrium point of the coupling spring. ");
    return -1;
  }

  AdmittanceController admittance_controller(nh, frequency,
                                             cmd_topic_platform,
                                             state_topic_platform,
                                             cmd_topic_arm,
                                             topic_arm_twist_world,
                                             topic_wrench_u_e,
                                             topic_wrench_u_c,
                                             state_topic_arm,
                                             wrench_topic, 
					     wrench_control_topic,
                                             M_p, M_a, D, D_p, D_a, K, d_e);
  admittance_controller.run();

  return 0;
}