/*
 * move_pos.cpp
 *
 *  Created on: Nov 30, 2013
 *      Author: martin
 */


#include <ros/ros.h>

#include <moveit/move_group_interface/move_group.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;

/**
 * Moves the arm to a position relative to the current position
 * according to parameters x,y and z
 *
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char **argv) {

	if(argc < 4) {
		ROS_ERROR("Usage: %s x y z (relative positions in m)", argv[0]);
		return EXIT_FAILURE;
	}

	ros::init(argc, argv, "move_pos");
	// this is necessary, because otherwise the calls to the movegroup interface will
	// not return, as they use service calls.
	ros::AsyncSpinner spinner(1);
	spinner.start();

	ROS_INFO("Connecting to movegroup interface for right arm");

	// Create MoveGroup for one of the planning groups
	move_group_interface::MoveGroup move_group("rightArm");
	move_group.setPlanningTime(5.0);
	move_group.setPoseReferenceFrame("world_link");

	double x,y,z;

	x = atof(argv[1]);
	y = atof(argv[2]);
	z = atof(argv[3]);

	geometry_msgs::PoseStamped pose = move_group.getCurrentPose();
	pose.pose.position.x += x;
	pose.pose.position.y += y;
	pose.pose.position.z += z;

	ROS_INFO("Setting position target");

	if(!move_group.setPoseTarget(pose)) {
		ROS_ERROR("Setting relative target position failed!");
		return EXIT_FAILURE;
	}

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
