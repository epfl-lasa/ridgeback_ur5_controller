#ifndef CARTESIAN_VELOCITY_CONTROLLER_SIM_H
#define CARTESIAN_VELOCITY_CONTROLLER_SIM_H

#include <ros/node_handle.h>
#include <urdf/model.h>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>
#include "ur_modern_driver/ur_hardware_interface.h"
#include <realtime_tools/realtime_publisher.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <control_msgs/JointControllerState.h>
#include <std_msgs/Float64.h>
#include <control_msgs/JointControllerState.h>
#include <kdl/chainiksolvervel_pinv.hpp>

#include "ur5_cartesian_velocity_control/kinematic_chain_controller_base.h"

namespace ros_control_ur
{

class CartesianVelocityControllerSim:
    public controller_interface::
        KinematicChainControllerBase<hardware_interface::PositionJointInterface>
{
public:

  CartesianVelocityControllerSim();
  ~CartesianVelocityControllerSim();

  /** \brief The init function is called to initialize the controller from a
   * non-realtime thread with a pointer to the hardware interface, itself,
   * instead of a pointer to a RobotHW.
   *
   * \param robot The specific hardware interface used by this controller.
   *
   * \param n A NodeHandle in the namespace from which the controller
   * should read its configuration, and where it should set up its ROS
   * interface.
   *
   * \returns True if initialization was successful and the controller
   * is ready to be started.
   */
  bool init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n);

  /** \brief This is called from within the realtime thread just before the
   * first call to \ref update
   *
   * \param time The current time
   */
  void starting(const ros::Time& time);

  /*!
   * \brief Issues commands to the joint. Should be called at regular intervals
   */
  void update(const ros::Time& time, const ros::Duration& period);

  /**
   * \brief Print debug info to console
   */
  void printDebug();

private:
  int loop_count_;
  ros::Subscriber sub_command_;

  bool sim_; // Indicates if the controller is running in simulation or on the real hardware

  KDL::Twist       x_dt_des_;      // Desired end-effector velocity
  KDL::JntArray       q_dt_cmd_;      // Desired joint velocity

  boost::shared_ptr<KDL::ChainIkSolverVel_pinv>       ik_vel_solver_;
};

} // namespace

#endif
