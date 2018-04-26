#include "ros/ros.h"
#include "AdmittanceController.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "admittance_controller_node");

  ros::NodeHandle nh;
  double frequency = 100.0;

  // Parameters
  std::string topic_arm_state;
  std::string topic_arm_command;
  std::string topic_platform_state;
  std::string topic_platform_command;
  std::string topic_external_wrench;
  std::string topic_control_wrench;
  std::string topic_equilibrium_desired;
  std::string topic_equilibrium_real;
  std::string topic_ds_velocity;
  std::string topic_external_wrench_arm_frame;
  std::string topic_control_external_arm_frame;
  std::string topic_admittance_ratio;
  std::string topic_arm_pose_world;
  std::string topic_arm_twist_world;

  std::vector<double> M_p;
  std::vector<double> M_a;
  std::vector<double> D;
  std::vector<double> D_p;
  std::vector<double> D_a;
  std::vector<double> K;
  std::vector<double> d_e;
  std::vector<double> workspace_limits;

  double arm_max_vel;
  double arm_max_acc;
  double platform_max_vel;
  double platform_max_acc;

  double wrench_filter_factor;
  double force_dead_zone_thres;
  double torque_dead_zone_thres;



  // LOADING PARAMETERS FROM THE ROS SERVER

  // Topic names
  if (!nh.getParam("topic_arm_state", topic_arm_state)) {
    ROS_ERROR("Couldn't retrieve the topic name for the state of the arm.");
    return -1;
  }

  if (!nh.getParam("topic_arm_command", topic_arm_command)) {
    ROS_ERROR("Couldn't retrieve the topic name for commanding the arm.");
    return -1;
  }

  if (!nh.getParam("topic_platform_state", topic_platform_state)) {
    ROS_ERROR("Couldn't retrieve the topic name for the state of the platform.");
    return -1;
  }

  if (!nh.getParam("topic_platform_command", topic_platform_command)) {
    ROS_ERROR("Couldn't retrieve the topic name for commanding the platform.");
    return -1;
  }

  if (!nh.getParam("topic_external_wrench", topic_external_wrench)) {
    ROS_ERROR("Couldn't retrieve the topic name for the force/torque sensor.");
    return -1;
  }

  if (!nh.getParam("topic_control_wrench", topic_control_wrench)) {
    ROS_ERROR("Couldn't retrieve the topic name for the control wrench.");
    return -1;
  }

  if (!nh.getParam("topic_equilibrium_desired", topic_equilibrium_desired)) {
    ROS_ERROR("Couldn't retrieve the topic name for the desired equilibrium point.");
    return -1;
  }

  if (!nh.getParam("topic_equilibrium_real", topic_equilibrium_real)) {
    ROS_ERROR("Couldn't retrieve the topic name for the  real equilibrium point.");
    return -1;
  }

  if (!nh.getParam("topic_ds_velocity", topic_ds_velocity)) {
    ROS_ERROR("Couldn't retrieve the topic name for the DS velocity.");
    return -1;
  }

  if (!nh.getParam("topic_external_wrench_arm_frame", topic_external_wrench_arm_frame)) {
    ROS_ERROR("Couldn't retrieve the topic name for the external wrench in the arm frame.");
    return -1;
  }

  if (!nh.getParam("topic_control_external_arm_frame", topic_control_external_arm_frame)) {
    ROS_ERROR("Couldn't retrieve the topic name for the control wrench in the arm frame.");
    return -1;
  }

  if (!nh.getParam("topic_admittance_ratio", topic_admittance_ratio)) {
    ROS_ERROR("Couldn't retrieve the topic name for the admittance ratio.");
    return -1;
  }

  if (!nh.getParam("topic_arm_pose_world", topic_arm_pose_world)) {
    ROS_ERROR("Couldn't retrieve the topic name for the EE pose in the world frame.");
    return -1;
  }

  if (!nh.getParam("topic_arm_twist_world", topic_arm_twist_world)) {
    ROS_ERROR("Couldn't retrieve the topic name for the EE twist in the world frame.");
    return -1;
  }


  // ADMITTANCE PARAMETERS
  if (!nh.getParam("mass_platform", M_p)) {
    ROS_ERROR("Couldn't retrieve the desired mass platform.");
    return -1;
  }

  if (!nh.getParam("mass_arm", M_a)) {
    ROS_ERROR("Couldn't retrieve the desired mass of the arm.");
    return -1;
  }

  if (!nh.getParam("damping_coupling", D)) {
    ROS_ERROR("Couldn't retrieve the desired damping of the coupling.");
    return -1;
  }

  if (!nh.getParam("damping_platform", D_p)) {
    ROS_ERROR("Couldn't retrieve the desired damping of the platform.");
    return -1;
  }

  if (!nh.getParam("damping_arm", D_a)) {
    ROS_ERROR("Couldn't retrieve the desired damping of the arm.");
    return -1;
  }

  if (!nh.getParam("stiffness_coupling", K)) {
    ROS_ERROR("Couldn't retrieve the desired stiffness of the coupling.");
    return -1;
  }

  if (!nh.getParam("equilibrium_point_spring", d_e)) {
    ROS_ERROR("Couldn't retrieve the desired equilibrium of the spring.");
    return -1;
  }



  // SAFETY PARAMETERS
  if (!nh.getParam("workspace_limits", workspace_limits)) {
    ROS_ERROR("Couldn't retrieve the limits of the workspace.");
    return -1;
  }

  if (!nh.getParam("arm_max_vel", arm_max_vel)) {
    ROS_ERROR("Couldn't retrieve the max velocity for the arm.");
    return -1;
  }

  if (!nh.getParam("arm_max_acc", arm_max_acc)) {
    ROS_ERROR("Couldn't retrieve the max acceleration for the arm.");
    return -1;
  }

  if (!nh.getParam("platform_max_vel", platform_max_vel)) {
    ROS_ERROR("Couldn't retrieve the max velocity for the platform.");
    return -1;
  }

  if (!nh.getParam("platform_max_acc", platform_max_acc)) {
    ROS_ERROR("Couldn't retrieve the max acceleration for the platform.");
    return -1;
  }



  // FORCE/TORQUE-SENSOR PARAMETERS
  if (!nh.getParam("wrench_filter_factor", wrench_filter_factor)) {
    ROS_ERROR("Couldn't retrieve the desired wrench filter factor.");
    return -1;
  }

  if (!nh.getParam("force_dead_zone_thres", force_dead_zone_thres)) {
    ROS_ERROR("Couldn't retrieve the desired force_dead_zone threshold.");
    return -1;
  }

  if (!nh.getParam("torque_dead_zone_thres", torque_dead_zone_thres)) {
    ROS_ERROR("Couldn't retrieve the desired torque_dead_zone threshold. ");
    return -1;
  }



  // Constructing the controller
  AdmittanceController admittance_controller(
    nh,
    frequency,
    topic_platform_command,
    topic_platform_state,
    topic_arm_command,
    topic_arm_pose_world,
    topic_arm_twist_world,
    topic_external_wrench_arm_frame,
    topic_control_external_arm_frame,
    topic_arm_state,
    topic_external_wrench,
    topic_control_wrench,
    topic_admittance_ratio,
    topic_equilibrium_desired,
    topic_equilibrium_real,
    topic_ds_velocity,
    M_p, M_a, D, D_p, D_a, K, d_e,
    workspace_limits,
    arm_max_vel, arm_max_acc,
    platform_max_vel, platform_max_acc,
    wrench_filter_factor,
    force_dead_zone_thres,
    torque_dead_zone_thres);

  // Running the controller
  admittance_controller.run();

  return 0;
}
