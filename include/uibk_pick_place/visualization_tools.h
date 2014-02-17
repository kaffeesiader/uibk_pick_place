/*
 * visualization_tools.h
 *
 *  Created on: Nov 8, 2013
 *      Author: martin
 */

#ifndef VISUALIZATION_TOOLS_H_
#define VISUALIZATION_TOOLS_H_

// Rviz
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/ColorRGBA.h>

#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseArray.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_interaction/robot_interaction.h>
#include <moveit_msgs/RobotTrajectory.h>

using namespace std;

namespace uibk_pick_place {

static const string ROBOT_DESCRIPTION="robot_description";
static const string COLLISION_TOPIC = "/collision_object";
static const string ATTACHED_COLLISION_TOPIC = "/attached_collision_object";
static const string RVIZ_MARKER_TOPIC = "/end_effector_marker";

enum rviz_colors { RED, GREEN, BLUE, GREY, WHITE, ORANGE };

/**
 * Contains some convenience methods to add/remove objects from the current planning
 * scene and to visualize arrows, text, gripper positions...
 *
 */
class VisualizationTools {

private:

	// A shared node handle
	ros::NodeHandle nh_;

	// ROS publishers
	ros::Publisher pub_rviz_marker_; // for rviz visualization markers
	ros::Publisher pub_collision_obj_; // for MoveIt collision objects
	ros::Publisher pub_attach_collision_obj_; // for MoveIt attached objects
	ros::Publisher pub_display_path_; // for MoveIt trajectories

	 // Pointer to a Planning Scene Monitor
	planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

	// Strings
	string marker_topic_; // topic to publish to rviz
	string ee_group_name_; // end effector group name
	string planning_group_name_; // planning group we are working with
	string base_link_; // name of base link of robot
	string ee_parent_link_; // parent link of end effector, loaded from MoveIt!

	// Duration to have Rviz markers persist, 0 for infinity
	ros::Duration marker_lifetime_;

	// End Effector Markers
	visualization_msgs::MarkerArray ee_marker_array_;
	tf::Pose tf_root_to_link_;
	geometry_msgs::Pose grasp_pose_to_eef_pose_; // Convert generic grasp pose to this end effector's frame of reference
	std::vector<geometry_msgs::Pose> marker_poses_;

	// Whether to actually publish to rviz or not
	bool muted_;

	// Cached Rviz markers
	visualization_msgs::Marker arrow_marker_;
	visualization_msgs::Marker sphere_marker_;
	visualization_msgs::Marker block_marker_;
	visualization_msgs::Marker text_marker_;
	visualization_msgs::Marker cylinder_marker_;

public:
	/**
	 * \brief Constructor with planning scene
	 */
	VisualizationTools(std::string base_link, planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor,
			std::string marker_topic = RVIZ_MARKER_TOPIC);
	/**
	 * \brief Constructor w/o planning scene passed in
	 */
	VisualizationTools(string base_link = "world_link", string marker_topic = RVIZ_MARKER_TOPIC);
	/**
	 * \brief Deconstructor
	 */
	~VisualizationTools();
	/**
	 * \brief Pre-load rviz markers for better efficiency
	 */
	void loadRvizMarkers();

	void attachCO(const std::string& name);
	/**
	 * \brief Remove a collision object from the planning scene
	 * \param Name of object
	 */
	void cleanupCO(const std::string& name);

	void cleanupACO(const std::string& name);
	/**
	 * @brief Get the planning scene monitor that this class is using
	 * @param planning_scene_monitor
	 * @return true if successful
	 */
	planning_scene_monitor::PlanningSceneMonitorPtr getPlanningSceneMonitor() {
		if (!planning_scene_monitor_) {
			ROS_ERROR_STREAM_NAMED("temp", "loading planning scene");

			loadPlanningSceneMonitor();
		}

		return planning_scene_monitor_;
	}
	/**
	 * \brief Set the name of the end effector
	 */
	void setEEGroupName(const string& ee_group_name) {
		ee_group_name_ = ee_group_name;
	}
	/**
	 * \brief Set the name of the end effector parent link
	 */
	void setEEParentLink(const string &ee_parent_link) {
		ee_parent_link_ = ee_parent_link;
	}
	/**
	 * \brief Provide the name of the planning group moveit will use
	 */
	void setPlanningGroupName(const string& planning_group_name) {
		planning_group_name_ = planning_group_name;
	}
	/**
	 * \brief Set this class to not actually publish anything to Rviz.
	 * \param muted true if verbose
	 */
	void setMuted(bool muted) {
		muted_ = muted;
	}
	/**
	 * \brief Return if we are in verbose mode
	 */
	bool isMuted() {
		return muted_;
	}
	/**
	 * \brief Load a planning scene monitor if one was not passed into the constructor
	 * \return true if successful in loading
	 */
	bool loadPlanningSceneMonitor();
	/**
	 * \brief Call this once at begining to load the robot marker
	 * \return true if it is successful
	 */
	bool loadEEMarker();
	/**
	 * \brief Publish an end effector to rviz
	 * \return true if it is successful
	 */
	bool publishEEMarkers(const geometry_msgs::Pose &pose, const rviz_colors &color = WHITE, const std::string &ns="end_effector");
	/**
	 * \brief Publish an marker of an arrow to rviz
	 * \return true if it is successful
	 */
	bool publishArrow(const geometry_msgs::Pose &pose, const rviz_colors color = BLUE);
	/**
	 * \brief Publish an marker of a block to Rviz
	 * \return true if it is successful
	 */
	bool publishBlock(const geometry_msgs::Pose &pose, const double &size, const bool isRed);

	/**
	 * \brief Publish an marker of a cylinder to Rviz
	 * \return true if it is successful
	 */
	bool publishCylinder(const geometry_msgs::Pose &pose, const double &height, const double &radius, const bool isRed);
	/**
	 * \brief Publish an marker of a text to Rviz
	 * \return true if it is successful
	 */
	bool publishText(const geometry_msgs::Pose &pose, const string &text, const rviz_colors &color = WHITE);

	void publishCollisionBlock(const geometry_msgs::Pose& pose, const string& name, double size);

	void publishCollisionBlock(const geometry_msgs::Pose& pose, double width, double height, double depth, const string& name);

	void publishCollisionCylinder(const geometry_msgs::Pose& pose, const string& name, double height, double radius);
	/**
	 * \brief Animate trajectory in rviz
	 * \param trajectory_msg the actual plan
	 * \param waitTrajectory whether we need to wait for the animation to complete
	 * \return true if no errors
	 */
	bool publishTrajectoryPath(const moveit_msgs::RobotTrajectory& trajectory_msg, bool waitTrajectory);
	/**
	 * \brief Get the RGB value of standard colors
	 * \param color - a enum pre-defined name of a color
	 * \return the RGB message equivalent
	 */
	std_msgs::ColorRGBA getColor(const rviz_colors &color);
};

typedef boost::shared_ptr<VisualizationTools> VisualizationToolsPtr;

}


#endif /* VISUALIZATION_TOOLS_H_ */
