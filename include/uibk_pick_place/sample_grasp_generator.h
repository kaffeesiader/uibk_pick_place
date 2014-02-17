/*
 * sample_grasp_generator.h
 *
 *  Created on: Nov 8, 2013
 *      Author: martin
 */

#ifndef SAMPLE_GRASP_GENERATOR_H_
#define SAMPLE_GRASP_GENERATOR_H_

// ROS
#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>
#include <geometry_msgs/PoseArray.h>
// #include <sensor_msgs/JointState.h>
#include <manipulation_msgs/Grasp.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

// Rviz
#include <uibk_pick_place/visualization_tools.h>

// C++
#include <math.h>
#define _USE_MATH_DEFINES

using namespace std;

namespace uibk_pick_place {

struct RobotGraspData {
	RobotGraspData() :
			// Fill in default values where possible:
			base_link_("/world_link"),
			grasp_depth_(0.12),
			angle_resolution_(16),
			approach_retreat_desired_dist_(0.6),
			approach_retreat_min_dist_(0.4),
			object_width(0.05),
			object_height(0.2) {
	}
	sensor_msgs::JointState pre_grasp_posture_;
	sensor_msgs::JointState grasp_posture_;
	std::string base_link_; // name of global frame with z pointing up
	std::string ee_parent_link_; // the last link in the kinematic chain before the end effector, e.g. "/gripper_roll_link"
	double grasp_depth_; // distance from center point of object to end effector
	int angle_resolution_; // generate grasps at PI/angle_resolution increments
	double approach_retreat_desired_dist_; // how far back from the grasp position the pregrasp phase should be
	double approach_retreat_min_dist_; // how far back from the grasp position the pregrasp phase should be at minimum
	double object_width; // for visualization
	double object_height;
};

class SampleGraspGenerator {

private:
	// class for publishing stuff to rviz
	VisualizationToolsPtr rviz_tools_;

	// Transform from frame of box to global frame
	Eigen::Affine3d global_transform_;

	// Create grasp positions in one axis
	bool generateAxisGrasps(vector<manipulation_msgs::Grasp>& possible_grasps, const RobotGraspData& grasp_data);

public:

	SampleGraspGenerator(VisualizationToolsPtr visualization_tools);
	~SampleGraspGenerator();

	// Create all possible grasp positions for a block
	bool generateGrasps(const geometry_msgs::Pose& pose, const RobotGraspData& grasp_data,
			vector<manipulation_msgs::Grasp>& possible_grasps);

	/**
	 * \brief Show all grasps in Rviz
	 * \param possible_grasps
	 * \param object_pose
	 * \param grasp_data - custom settings for a robot's geometry
	 */
	void visualizeGrasps(const vector<manipulation_msgs::Grasp>& possible_grasps,
			const geometry_msgs::Pose& object_pose, const RobotGraspData& grasp_data);

	static void printGraspData(const RobotGraspData& data) {
		ROS_INFO_STREAM_NAMED("grasp", "ROBOT GRASP DATA DEBUG OUTPUT ---------------------");
		ROS_INFO_STREAM_NAMED("grasp", "Base Link: " << data.base_link_);
		ROS_INFO_STREAM_NAMED("grasp", "EE Parent Link: " << data.ee_parent_link_);
		ROS_INFO_STREAM_NAMED("grasp", "Grasp Depth: " << data.grasp_depth_);
		ROS_INFO_STREAM_NAMED("grasp", "Angle Resolution: " << data.angle_resolution_);
		ROS_INFO_STREAM_NAMED("grasp", "Approach Retreat Desired Dist: " << data.approach_retreat_desired_dist_);
		ROS_INFO_STREAM_NAMED("grasp", "Approach Retreat Min Dist: " << data.approach_retreat_min_dist_);
		ROS_INFO_STREAM_NAMED("grasp", "Pregrasp Posture: \n" << data.pre_grasp_posture_);
		ROS_INFO_STREAM_NAMED("grasp", "Grasp Posture: \n" << data.grasp_posture_);
		ROS_INFO_STREAM_NAMED("grasp", "---------------------------------------------------\n");
	}

};

typedef boost::shared_ptr<SampleGraspGenerator> SampleGraspGeneratorPtr;

}

#endif /* SAMPLE_GRASP_GENERATOR_H_ */
