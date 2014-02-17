/*
 * move_goal.cpp
 *
 *  Created on: Nov 30, 2013
 *      Author: martin
 */

#include <ros/ros.h>

#include <moveit/move_group_interface/move_group.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;

/**
 * Moves the end effector to given pose.
 * Takes x, y, z and roll, pitch and yaw values as parameters.
 *
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char **argv) {

	if(argc < 4) {
		ROS_ERROR("Usage: %s x y z", argv[0]);
		return EXIT_FAILURE;
	}

	ros::init(argc, argv, "move_goal_constraint");
	// this is necessary, because otherwise the calls to the movegroup interface will
	// not return, as they use service calls.
	ros::AsyncSpinner spinner(1);
	spinner.start();

	ROS_INFO("Connecting to movegroup interface for right arm");

	// Create MoveGroup for one of the planning groups
	move_group_interface::MoveGroup move_group("rightArm");
	move_group.setPoseReferenceFrame("world_link");
	move_group.setPlannerId("RRTConnectkConfigDefault");

	double x,y,z,t;

	if(argc > 4) {
		t = atof(argv[4]);
	} else {
		t = 5.0;
	}

	ROS_INFO("Setting allowed planning time to %.1fs", t);
	move_group.setPlanningTime(t);

	x = atof(argv[1]);
	y = atof(argv[2]);
	z = atof(argv[3]);

	ROS_INFO("Setting pose target");

	if(!move_group.setPositionTarget(x, y, z)) {
		ROS_ERROR("Setting position target failed!");
		return EXIT_FAILURE;
	}

//	moveit_msgs::Constraints constraints;
//	// define the orientation constraint
//	moveit_msgs::OrientationConstraint oc;
//
//	oc.header.frame_id = "world_link";
//	oc.link_name = "right_arm_7_link";
//	oc.orientation = move_group.getCurrentPose().pose.orientation;
//	oc.absolute_x_axis_tolerance = M_PI;
//	oc.absolute_y_axis_tolerance = M_PI;
//	oc.absolute_z_axis_tolerance = M_PI;
//	oc.weight = 1;
//
//	constraints.name = "Orientation constraints";
//	constraints.orientation_constraints.push_back(oc);
//
//	move_group.setPathConstraints(constraints);

	ROS_INFO("Calling planning service");
	move_group_interface::MoveGroup::Plan plan;
	if(!move_group.plan(plan)) {
		ROS_ERROR("Planning failed!");
		return EXIT_FAILURE;
	}

	ROS_INFO("Motion plan calculated - executing trajectory.");

	if(!move_group.execute(plan)) {
		ROS_ERROR("Trajectory execution failed!");
		return EXIT_FAILURE;
	}

	ROS_INFO("Pose target reached");

	geometry_msgs::PoseStamped pose_msg = move_group.getCurrentPose();
	ROS_INFO("Calculated position (frame_id = %s): x=%.2f, y=%.2f, z=%.2f", pose_msg.header.frame_id.c_str(), pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z);

	return EXIT_SUCCESS;

}
