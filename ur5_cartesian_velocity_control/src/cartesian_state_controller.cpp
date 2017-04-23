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

  // get publishing period
  if (!n.getParam("publish_rate", publish_rate_)){
      ROS_ERROR("Parameter 'publish_rate' not set");
      return false;
  }
  realtime_pub_.reset(new realtime_tools::RealtimePublisher
                      <cartesian_state_msgs::PoseTwist>(n, "ee_state", 4));

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
  last_publish_time_ = time;
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

  // Limit rate of publishing
  if (publish_rate_ > 0.0 && last_publish_time_
       + ros::Duration(1.0/publish_rate_) < time) {

    // try to publish
    if (realtime_pub_->trylock()) {
      // we're actually publishing, so increment time
      last_publish_time_ = last_publish_time_
                           + ros::Duration(1.0/publish_rate_);

      // populate message
      realtime_pub_->msg_.header.stamp = time;
      tf::poseKDLToMsg(x_, realtime_pub_->msg_.pose);
      tf::twistKDLToMsg(x_dot_.GetTwist(), realtime_pub_->msg_.twist);

      realtime_pub_->unlockAndPublish();
    }
  }

}

} // controller_interface

// Register controllers with the PLUGINLIB_EXPORT_CLASS macro to enable dynamic
// loading with the controller manager
PLUGINLIB_EXPORT_CLASS(controller_interface::CartesianStateController, controller_interface::ControllerBase)
