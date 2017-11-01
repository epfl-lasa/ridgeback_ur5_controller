#include "ObjectTracker.h"

ObjectTracker::ObjectTracker(
    ros::NodeHandle &n,
    double frequency,
    std::string frame_platform_base,
    std::string frame_arm_base,
    std::string frame_robot_calibration,
    std::string frame_robot_endeffector,
    std::string frame_robot_object,
    std::string frame_mocap_calibration,
    std::string frame_mocap_endeffector,
    std::string frame_mocap_object)
	: nh_(n),
	  loop_rate_(frequency),
	  frame_platform_base_(frame_platform_base),
	  frame_arm_base_(frame_arm_base),
	  frame_robot_calibration_(frame_robot_calibration),
	  frame_robot_endeffector_(frame_robot_endeffector),
	  frame_robot_object_(frame_robot_object),
	  frame_mocap_calibration_(frame_mocap_calibration),
	  frame_mocap_endeffector_(frame_mocap_endeffector),
	  frame_mocap_object_(frame_mocap_object) {

	ROS_INFO_STREAM("ObjectTracker is created at: " << nh_.getNamespace() << " with freq: " << frequency << "Hz");

	tf::StampedTransform transform_temp;
	tf::StampedTransform transform_ee_pbase;
	tf::StampedTransform transform_pbase_calib;
	tf::StampedTransform transform_mocap_ee_calib;
	tf::StampedTransform transform_robot_ee_calib;

	bool ready_base = false;
	bool ready_endeffector = false;
	bool ready_calibration = false;
	bool ready_mocap = false;

	quat_offset_ = tf::createQuaternionFromRPY(0, 0, 0);


	ROS_INFO_STREAM("Make sure the z-axis for both "
	                << frame_mocap_endeffector_ << " and "
	                << frame_robot_endeffector_ << " is pointing up.");

	ROS_INFO_STREAM("Make sure markers are visibles: "
	                << frame_mocap_calibration_ << " and "
	                << frame_mocap_endeffector_);

	ROS_INFO("If ready, press ENTER ...");

	std::cin.get();


	while (nh_.ok() && (!ready_base || !ready_endeffector || !ready_calibration || !ready_mocap)) {
		try {
			listener_.lookupTransform(frame_platform_base_, frame_platform_base_,
			                          ros::Time(0), transform_temp);

			ready_base = true;

			time_diff_ = transform_temp.stamp_ - ros::Time::now();
			ROS_WARN_STREAM_THROTTLE(10, "Time difference:" << time_diff_ );
		}
		catch (tf::TransformException ex) {
			ROS_WARN_STREAM_THROTTLE(1, "Waiting for " <<  frame_platform_base_);
		}

		try {
			listener_.lookupTransform(frame_robot_endeffector_, frame_platform_base_,
			                          ros::Time(0), transform_ee_pbase);
			ready_endeffector = true;
		}
		catch (tf::TransformException ex) {
			ROS_WARN_STREAM_THROTTLE(1, "Waiting for TF from: " <<
			                         frame_robot_endeffector_ << " to: "
			                         << frame_platform_base_);
		}

		try {
			listener_.lookupTransform(frame_platform_base_, frame_robot_calibration_,
			                          ros::Time(0), transform_pbase_calib);
			ready_calibration = true;
		}
		catch (tf::TransformException ex) {
			ROS_WARN_STREAM_THROTTLE(1, "Waiting for TF from: " <<
			                         frame_platform_base_ << " to: "
			                         << frame_robot_calibration_);
		}

		try {
			listener_.lookupTransform(frame_mocap_endeffector_, frame_mocap_calibration_,
			                          ros::Time(0), transform_mocap_ee_calib);
			ready_mocap = true;
		}
		catch (tf::TransformException ex) {
			ROS_WARN_STREAM_THROTTLE(1, "Waiting for TF from: " <<
			                         frame_mocap_endeffector_ << " to: "
			                         << frame_mocap_calibration_);
		}

		ros::spinOnce();
		loop_rate_.sleep();
	}

	// this transformation cannot be listened to directly due to time difference problems
	transform_robot_ee_calib.setData(transform_ee_pbase * transform_pbase_calib);


	tf::Vector3 origin1 = transform_mocap_ee_calib.getOrigin();
	tf::Vector3 origin2 = transform_robot_ee_calib.getOrigin();


	if (origin1.getZ()*origin2.getZ() < 0) {
		ROS_WARN("It seems that the z-axes are not matching!");
		ROS_INFO_STREAM("calibration point in robot: " << origin1.getX() << " " << origin1.getY() << " " << origin1.getZ() );
		ROS_INFO_STREAM("calibration point in mocap: " << origin2.getX() << " " << origin2.getY() << " " << origin2.getZ() );
	}

	// assuming the two frames matching in z-axis
	origin1.setZ(0);
	origin2.setZ(0);


	tf::Vector3 xyz = origin1.cross(origin2);
	xyz.normalized();
	double w = 1.0 * acos(origin1.dot(origin2) / (origin1.length() * origin2.length()));

	tf::Quaternion q(xyz, w);
	q.normalize();
	quat_offset_ = q;

	ROS_INFO_STREAM("Calibraton is finished. Rotation offset: " << quat_offset_.getAngleShortestPath() << "[rad]");

}


void ObjectTracker::Run() {

	tf::StampedTransform transform;
	tf::StampedTransform transform_armbase_endeffector;

	transform.setIdentity();
	transform_armbase_endeffector.setIdentity();


	while (nh_.ok()) {

		try {
			listener_.lookupTransform(frame_arm_base_, frame_robot_endeffector_,
			                          ros::Time(0), transform_armbase_endeffector);
		}
		catch (tf::TransformException ex) {
			ROS_WARN_STREAM_THROTTLE(1, "Waiting for TF from: " <<
			                         frame_arm_base_ << " to: "
			                         << frame_robot_endeffector_);
		}

		try {
			listener_.lookupTransform(frame_mocap_endeffector_, frame_mocap_object_,
			                          ros::Time(0), transform);

			tf::Vector3 correct_position = tf::quatRotate(quat_offset_, transform.getOrigin());

			transform.setOrigin(correct_position);

			// br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_robot_static_, "object"));
			transform.setData(transform_armbase_endeffector * transform );

			transform.stamp_ = ros::Time::now() + time_diff_;
			transform.frame_id_ = frame_arm_base_;
			transform.child_frame_id_ = frame_robot_object_;
			brodcaster_.sendTransform(transform);

		}
		catch (tf::TransformException ex) {
			ROS_WARN_STREAM_THROTTLE(1, "Waiting for TF from: "
			                         << frame_mocap_endeffector_ << " to: "
			                         << frame_mocap_object_ );
		}

		ros::spinOnce();
		loop_rate_.sleep();

	}
}

