/*
 * eigen_tests.cpp
 *
 *  Created on: Nov 10, 2013
 *      Author: martin
 */

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>
//#include <uibk_pick_place/visualization_tools.h>
#include <geometry_msgs/Pose.h>
#include <stdio.h>
#include <iostream>
// #include <math.h>

using namespace std;

int main(int argc, char **argv) {

	ros::init(argc, argv, "eigen_tests");

	tfScalar x = 0.64085650444031;
	tfScalar y = 0.64085638523102;
	tfScalar z = 0.29883641004562;
	tfScalar w = 0.29883599281311;

	tf::Quaternion quad(x, y, z, w);
	double roll, pitch, yaw;

	tf::Matrix3x3(quad).getRPY(roll, pitch, yaw);
	ROS_INFO("roll: %0.4f, pitch: %0.4f, yaw: %0.4f\n", roll, pitch, yaw);

	int numCPU = sysconf( _SC_NPROCESSORS_ONLN );
	ROS_INFO("Computer contains %d CPU's", numCPU);

//	ros::AsyncSpinner spinner(1);
//	spinner.start();
//
//	VisualizationTools rviz_tools("world_link");
//	rviz_tools.setEEGroupName("rightGripper");
//	rviz_tools.setPlanningGroupName("rightArm");
//
//	geometry_msgs::Pose op;
//
//	op.position.x = 0;
//	op.position.y = 0;
//	op.position.z = 0.1;
//
//	op.orientation.x = 0;
//	op.orientation.y = 0;
//	op.orientation.z = 0;
//	op.orientation.w = 1;
//
//	rviz_tools.publishCylinder(op, 0.2, 0.025, true);
//
//	double radius = 0.15; // distance from eef to object
//	double xb = 0.0;
//	double yb = 0.0;
//	double zb = 0.1;
//	double theta = M_PI; // Where the point is located around the cylinder
//	int resolution = 18;
//
//	for(int i = 0; i < resolution; ++i) {
//
//		xb = radius * -sin(theta);
//		yb = radius * cos(theta);
//
//
//		Eigen::Affine3d grasp_pose;
//
//		grasp_pose = Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitX()) *
//					 Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitY()) *
//					 Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitZ());
//
//		theta -= M_PI / resolution;
//
//		grasp_pose.translation() = Eigen::Vector3d(xb, yb, zb);
//		geometry_msgs::Pose pose;
//
//		tf::poseEigenToMsg(grasp_pose, pose);
//
//		cout << "publishing " << i + 1 << "th EEF pose." << endl;
//
//		rviz_tools.publishEEMarkers(pose, RED);
//
//		ros::WallDuration(1).sleep();
//	}
}


