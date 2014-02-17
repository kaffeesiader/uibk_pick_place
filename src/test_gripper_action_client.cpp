/*
 * test_gripper_action_client.cpp
 *
 *  Created on: Nov 9, 2013
 *      Author: martin
 */

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>

using namespace actionlib;
using namespace std;

int main(int argc, char **argv) {
	ros::init(argc, argv, "test_gripper");

	// create the action client
	// true causes the client to spin its own thread
	SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("/sdh_controller/follow_joint_trajectory/", true);

	ROS_INFO("Waiting for action server to start.");
	// wait for the action server to start
	ac.waitForServer(); //will wait for infinite time

	ROS_INFO("Action server started, sending goal.");
	// send a goal to the action
	control_msgs::FollowJointTrajectoryGoal goal;

	goal.trajectory.joint_names.push_back("sdh_finger_12_joint");
	goal.trajectory.joint_names.push_back("sdh_finger_13_joint");
	goal.trajectory.joint_names.push_back("sdh_finger_22_joint");
	goal.trajectory.joint_names.push_back("sdh_finger_23_joint");
	goal.trajectory.joint_names.push_back("sdh_knuckle_joint");
	goal.trajectory.joint_names.push_back("sdh_thumb_2_joint");
	goal.trajectory.joint_names.push_back("sdh_thumb_3_joint");

	trajectory_msgs::JointTrajectoryPoint point;

	if(argc > 1) {
		ROS_INFO("Movint to neutral position.");

		point.positions.push_back(0);
		point.positions.push_back(0);
		point.positions.push_back(0);
		point.positions.push_back(0);
		point.positions.push_back(0);
		point.positions.push_back(0);
		point.positions.push_back(0);
	} else {
		point.positions.push_back(-M_PI / 6);
		point.positions.push_back(M_PI / 9);
		point.positions.push_back(-M_PI / 6);
		point.positions.push_back(M_PI / 9);
		point.positions.push_back(M_PI / 4);
		point.positions.push_back(-M_PI / 6);
		point.positions.push_back(M_PI / 9);
	}

	point.time_from_start = ros::Duration(4);

	goal.trajectory.points.push_back(point);

	ac.sendGoal(goal);

	//wait for the action to return
	bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

	if (finished_before_timeout) {
		actionlib::SimpleClientGoalState state = ac.getState();
		ROS_INFO("Action finished: %s", state.toString().c_str());
	} else
		ROS_INFO("Action did not finish before the time out.");

	//exit
	return 0;
}

