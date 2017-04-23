#ifndef CARTESIAN_STATE_CONTROLLER_H
#define CARTESIAN_STATE_CONTROLLER_H

#include <ros/node_handle.h>
#include <cartesian_state_msgs/PoseTwist.h>
#include <hardware_interface/joint_state_interface.h>
#include <controller_interface/controller.h>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include "ur5_cartesian_velocity_control/kinematic_chain_controller_base.h"
#include <realtime_tools/realtime_publisher.h>

namespace controller_interface
{

/** \brief This class implements a cartesian state publisher
 * for ROS control.
 */

class CartesianStateController:
    public KinematicChainControllerBase<
        hardware_interface::JointStateInterface>
{
public:
  CartesianStateController() {}
  ~CartesianStateController() {}

  /** \brief The init function is called to initialize the controller from a
   * non-realtime thread with a pointer to the hardware interface, itself,
   * instead of a pointer to a RobotHW.
   *s
   * \param robot The specific hardware interface used by this controller.
   *
   * \param n A NodeHandle in the namespace from which the controller
   * should read its configuration, and where it should set up its ROS
   * interface.
   *
   * \returns True if initialization was successful and the controller
   * is ready to be started.
   */
  bool init(hardware_interface::JointStateInterface *robot, ros::NodeHandle &n);

  /** \brief This is called from within the realtime thread just before the
   * first call to \ref update
   *
   * \param time The current time
   */
  void starting(const ros::Time& time);

  /*!
   * \brief Issues commands to the joint. Called at regular intervals
   */
  void update(const ros::Time& time, const ros::Duration& period);

protected:
  ros::Publisher pub_state_;

  KDL::FrameVel x_dot_;
  KDL::Frame x_;

  boost::shared_ptr<KDL::ChainFkSolverVel> fk_vel_solver_;
  boost::shared_ptr<KDL::ChainFkSolverPos> fk_pos_solver_;

  boost::shared_ptr<realtime_tools::RealtimePublisher<
     cartesian_state_msgs::PoseTwist> > realtime_pub_;

  ros::Time last_publish_time_;
  double publish_rate_;
};

} // namespace controller_interface


#endif
