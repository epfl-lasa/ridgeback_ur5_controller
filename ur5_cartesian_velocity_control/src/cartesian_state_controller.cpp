#include <pluginlib/class_list_macros.h>
#include "ur5_cartesian_velocity_control/kinematic_chain_controller_base.h"
#include "ur5_cartesian_velocity_control/cartesian_state_controller.h"
#include "kdl_conversions/kdl_msg.h"


namespace controller_interface
{
/** \brief Initialize the kinematic chain for kinematics-based computation.
 *
 */

bool CartesianStateController::init(
    hardware_interface::JointStateInterface *robot, ros::NodeHandle &n) {

  KinematicChainControllerBase<hardware_interface::JointStateInterface>::
         init(robot, n);

  fk_vel_solver_.reset(new KDL::ChainFkSolverVel_recursive(kdl_chain_));
  fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));

  pub_state_ = n.advertise<geometry_msgs::Twist>("ee_pose", 1);
  pub_state_deriv_ = n.advertise<geometry_msgs::Pose>("ee_vel", 1);

  x_.p.Zero();
  x_.M.Identity();
  x_dot_.p.Zero();
  x_dot_.M.Identity();

  return true;
}

/** \brief This is called from within the realtime thread just before the
 * first call to \ref update
 *
 * \param time The current time
 */
void CartesianStateController::starting(const ros::Time& time) {
}

/*!
 * \brief Issues commands to the joint. Should be called at regular intervals
 */
void CartesianStateController::update(const ros::Time& time,
                                         const ros::Duration& period) {

  // Get joint positions
  for(std::size_t i=0; i < joint_handles_.size(); i++)
  {
    joint_msr_.q(i)         = joint_handles_[i].getPosition();
    joint_msr_.qdot(i)      = joint_handles_[i].getVelocity();
  }

  // Compute forward kinematics
  fk_vel_solver_->JntToCart(joint_msr_, x_dot_);
  fk_pos_solver_->JntToCart(joint_msr_.q, x_);

  tf::poseKDLToMsg(x_, msg_pose_);
  tf::twistKDLToMsg(x_dot_.GetTwist(), msg_twist_);

  pub_state_.publish(msg_pose_);
  pub_state_deriv_.publish(msg_twist_);
}

} // controller_interface

// Register controllers with the PLUGINLIB_EXPORT_CLASS macro to enable dynamic
// loading with the controller manager
PLUGINLIB_EXPORT_CLASS(controller_interface::CartesianStateController, controller_interface::ControllerBase)
