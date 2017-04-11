#include "ur5_cartesian_velocity_control/cartesian_velocity_controller_sim.h"
#include <pluginlib/class_list_macros.h>

namespace ros_control_ur
{

CartesianVelocityControllerSim::CartesianVelocityControllerSim(){}
CartesianVelocityControllerSim::~CartesianVelocityControllerSim(){}

/** \brief Initialize the kinematic chain for kinematics-based computation.
 *
 */
bool CartesianVelocityControllerSim::init(
    hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n) {

  KinematicChainControllerBase
      <hardware_interface::PositionJointInterface>::init(robot, n);

  ik_vel_solver_.reset(new KDL::ChainIkSolverVel_pinv(kdl_chain_));

  joint_msr_.resize(kdl_chain_.getNrOfJoints());
  q_dt_cmd_.resize(kdl_chain_.getNrOfJoints());

  return true;
}

/** \brief This is called from within the realtime thread just before the
 * first call to \ref update
 *
 * \param time The current time
 */
void CartesianVelocityControllerSim::starting(const ros::Time& time){
  for(std::size_t i=0; i < joint_handles_.size(); i++) {
    q_dt_cmd_(i) = 0.0;
  }
  x_dt_des_ = KDL::Twist::Zero();
}

/*!
 * \brief Issues commands to the joint. Should be called at regular intervals
 */
void CartesianVelocityControllerSim::update(const ros::Time& time,
                                         const ros::Duration& period){

  // Get joint positions
  for(std::size_t i=0; i < joint_handles_.size(); i++)
  {
    joint_msr_.q(i)         = joint_handles_[i].getPosition();
    joint_msr_.qdot(i)      = joint_handles_[i].getVelocity();
  }

  // Compute inverse kinematics velocity solver
  ik_vel_solver_->CartToJnt(joint_msr_.q, x_dt_des_, q_dt_cmd_);

  // Integrate qdot_cmd_ to get

  for(std::size_t i=0; i < joint_handles_.size(); i++) {
    joint_handles_[i].setCommand(q_dt_cmd_(i));
  }
}

/**
 * \brief Print debug info to console
 */
void CartesianVelocityControllerSim::printDebug() {

}

} // namespace

PLUGINLIB_EXPORT_CLASS(ros_control_ur::CartesianVelocityControllerSim, controller_interface::ControllerBase)
