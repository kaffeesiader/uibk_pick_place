/*
 * test_pick_place.cpp
 *
 *  Created on: Nov 6, 2013
 *      Author: martin
 */

#include <ros/ros.h>

#include <moveit/move_group_interface/move_group.h>
#include <uibk_pick_place/sample_grasp_generator.h>
#include <uibk_pick_place/visualization_tools.h> // simple tool for showing graspsp

#include <boost/scoped_ptr.hpp>

using namespace std;

namespace uibk_pick_place {


static const string BASE_LINK = "world_link";
static const string EE_PARENT_LINK = "right_arm_7_link";
static const string EE_GROUP_NAME = "sdh";
static const string OBJ_ID = "cylinder1";

static const string SUPPORT_SURFACE_NAME = "table_surface";
static const double SUPPORT_SURFACE_HEIGHT = 0.08;

static const double CYLINDER_HEIGHT = 0.25;
static const double CYLINDER_RADIUS = 0.04;

class TestPickPlace {

public:

	// grasp generator
	SampleGraspGeneratorPtr grasp_generator_;

	VisualizationToolsPtr visual_tools_;

	// data for generating grasps
	RobotGraspData grasp_data_;

	// our interface with MoveIt
	boost::scoped_ptr<move_group_interface::MoveGroup> move_group_;

	// which arm are we using
	std::string arm_;
	std::string planning_group_name_;

	// settings
	bool auto_reset_;
	int auto_reset_sec_;
	int pick_place_count_; // tracks how many pick_places have run

	TestPickPlace() :
			arm_("right"),
			planning_group_name_(arm_ + "Arm"),
			auto_reset_(false),
			auto_reset_sec_(4),
			pick_place_count_(0)
	{
		ros::NodeHandle nh;

		// Create MoveGroup for one of the planning groups
		move_group_.reset(new move_group_interface::MoveGroup(planning_group_name_));
		move_group_->setPlanningTime(30.0);


		// Load the Robot Viz Tools for publishing to rviz
		visual_tools_.reset(new VisualizationTools(BASE_LINK));
		visual_tools_->setEEGroupName(EE_GROUP_NAME);
		visual_tools_->setEEParentLink(EE_PARENT_LINK);
		visual_tools_->setPlanningGroupName(planning_group_name_);

		// Load grasp generator
		// grasp_data_ = loadRobotGraspData(arm_, CYLINDER_HEIGHT, CYLINDER_RADIUS); // Load robot specific data
		grasp_generator_.reset(new SampleGraspGenerator(visual_tools_));

		// Initialize grasp settings
		initGraspData(grasp_data_);

		// Let everything load
		ros::Duration(1.0).sleep();

	}

	bool start() {
		// ---------------------------------------------------------------------------------------------
		// Load hard coded poses
		geometry_msgs::Pose start_pose = getStartPose();
		geometry_msgs::Pose goal_pose = createGoalPose();

		// create table surface, obstacle and object to move...
		createEnvironment();

		bool found = false;
		while (!found && ros::ok()) {

			if (!pick(start_pose, OBJ_ID)) {
				ROS_ERROR_STREAM_NAMED("simple_pick_place", "Pick failed. Retrying.");
				visual_tools_->cleanupACO(OBJ_ID);
			} else {
				ROS_INFO_STREAM_NAMED("simple_pick_place", "Done with pick ---------------------------");
				found = true;
			}
		}

		ROS_INFO_STREAM_NAMED("simple_pick_place", "Waiting to put...");
		ros::Duration(0.5).sleep();

		bool placed = false;
		while (!placed && ros::ok()) {
			if (!place(goal_pose, OBJ_ID)) {
				ROS_ERROR_STREAM_NAMED("simple_pick_place", "Place failed.");
			} else {
				ROS_INFO_STREAM_NAMED("simple_pick_place", "Done with place");
				placed = true;
				//visual_tools_->cleanupACO(OBJ_ID);
			}
		}

		// Move to neutral position
		// move_to_neutral_position();

		ROS_INFO_STREAM_NAMED("uibk_pick_place", "Pick and place cycle complete ========================================= \n");

		return true;
	}

	void createEnvironment() {
		// Remove arbitrary existing objects
		visual_tools_->cleanupCO(SUPPORT_SURFACE_NAME);
		visual_tools_->cleanupCO(OBJ_ID);

		// create table surface (with some additional size...);
		double width = 2.30;
		double height = SUPPORT_SURFACE_HEIGHT;
		double depth = 1.2;

		// calculate the position of the table surface with respect to the origin
		geometry_msgs::Pose s_pose;

		s_pose.position.x = 0.6 - 0.029 - 0.05;
		s_pose.position.y = 1.15 - 0.3 - 0.04;
		s_pose.position.z = height / 2;

		s_pose.orientation.x = 0;
		s_pose.orientation.y = 0;
		s_pose.orientation.z = 0;
		s_pose.orientation.w = 1;

		visual_tools_->publishCollisionBlock(s_pose, width, height, depth, SUPPORT_SURFACE_NAME);

		// create an obstacle
		width = 0.20;
		height = 0.2;
		depth = 0.3;

		// calculate the position of the table surface with respect to the origin
		geometry_msgs::Pose o_pose;

		o_pose.position.x = 0.25;
		o_pose.position.y = 0.25;
		o_pose.position.z = height / 2 + SUPPORT_SURFACE_HEIGHT;

		o_pose.orientation.x = 0;
		o_pose.orientation.y = 0;
		o_pose.orientation.z = 0;
		o_pose.orientation.w = 1;

		visual_tools_->publishCollisionBlock(o_pose, width, height, depth, "obstacle");

		// create object to pick
		geometry_msgs::Pose start_pose = getStartPose();
		visual_tools_->publishCollisionCylinder(start_pose, OBJ_ID, CYLINDER_HEIGHT, CYLINDER_RADIUS);
	}

	void initGraspData(RobotGraspData& grasp_data) {

		// -------------------------------
		// Create pre-grasp posture (Gripper open)
		grasp_data.pre_grasp_posture_.header.frame_id = BASE_LINK;
		grasp_data.pre_grasp_posture_.header.stamp = ros::Time::now();
		// Name of joints:
		grasp_data.pre_grasp_posture_.name.resize(8);
		grasp_data.pre_grasp_posture_.name[0] = "right_sdh_knuckle_joint";
		grasp_data.pre_grasp_posture_.name[1] = "right_sdh_finger_12_joint";
		grasp_data.pre_grasp_posture_.name[2] = "right_sdh_finger_13_joint";
		grasp_data.pre_grasp_posture_.name[3] = "right_sdh_finger_21_joint";
		grasp_data.pre_grasp_posture_.name[4] = "right_sdh_finger_22_joint";
		grasp_data.pre_grasp_posture_.name[5] = "right_sdh_finger_23_joint";
		grasp_data.pre_grasp_posture_.name[6] = "right_sdh_thumb_2_joint";
		grasp_data.pre_grasp_posture_.name[7] = "right_sdh_thumb_3_joint";
		// Position of joints
		grasp_data.pre_grasp_posture_.position.resize(8);
		grasp_data.pre_grasp_posture_.position[0] = 0.0;
		grasp_data.pre_grasp_posture_.position[1] = -M_PI / 4;	// -30 degrees
		grasp_data.pre_grasp_posture_.position[2] = M_PI / 9;	// +20 degrees
		grasp_data.pre_grasp_posture_.position[3] = 0.0;
		grasp_data.pre_grasp_posture_.position[4] = -M_PI / 4;	// -30 degrees
		grasp_data.pre_grasp_posture_.position[5] = M_PI / 9;	// +20 degrees
		grasp_data.pre_grasp_posture_.position[6] = -M_PI / 4;	// -30 degrees
		grasp_data.pre_grasp_posture_.position[7] = M_PI / 9;	// +20 degrees

		// -------------------------------
		// Create grasp posture (Gripper closed)
		grasp_data.grasp_posture_.header.frame_id = BASE_LINK;
		grasp_data.grasp_posture_.header.stamp = ros::Time::now();
		// Name of joints:
		grasp_data.grasp_posture_.name.resize(8);
		grasp_data.grasp_posture_.name[0] = "right_sdh_knuckle_joint";
		grasp_data.grasp_posture_.name[1] = "right_sdh_finger_12_joint";
		grasp_data.grasp_posture_.name[2] = "right_sdh_finger_13_joint";
		grasp_data.grasp_posture_.name[3] = "right_sdh_finger_21_joint";
		grasp_data.grasp_posture_.name[4] = "right_sdh_finger_22_joint";
		grasp_data.grasp_posture_.name[5] = "right_sdh_finger_23_joint";
		grasp_data.grasp_posture_.name[6] = "right_sdh_thumb_2_joint";
		grasp_data.grasp_posture_.name[7] = "right_sdh_thumb_3_joint";
		// Position of joints
		grasp_data.grasp_posture_.position.resize(8);
		grasp_data.grasp_posture_.position[0] = 0;
		grasp_data.grasp_posture_.position[1] = -M_PI / 14;
		grasp_data.grasp_posture_.position[2] = M_PI / 6;	// +20 degrees
		grasp_data.grasp_posture_.position[3] = 0;
		grasp_data.grasp_posture_.position[4] = -M_PI / 14;
		grasp_data.grasp_posture_.position[5] = M_PI / 6;	// +20 degrees
		grasp_data.grasp_posture_.position[6] = -M_PI / 14;
		grasp_data.grasp_posture_.position[7] = M_PI / 6;	// +20 degrees

		// -------------------------------
		// Links
		grasp_data.base_link_ = BASE_LINK;
		grasp_data.ee_parent_link_ = EE_PARENT_LINK;

		// -------------------------------
		grasp_data.approach_retreat_desired_dist_ = 0.2; // 0.1;
		grasp_data.approach_retreat_min_dist_ = 0.10; // 0.001;

		// distance from center point of object to end effector
		grasp_data.grasp_depth_ = 0.19; // 0.1;

		grasp_data.object_height = CYLINDER_HEIGHT;
		grasp_data.object_width = CYLINDER_RADIUS * 2;

		// generate grasps at PI/2/angle_resolution increments
		grasp_data.angle_resolution_ = 9;

		// Debug
		// SampleGraspGenerator::printGraspData(grasp_data_);
	}

	geometry_msgs::Pose getStartPose() {
		geometry_msgs::Pose start_pose;

		// Position
		start_pose.position.x = 0.0;
		start_pose.position.y = 0.0;
		start_pose.position.z = CYLINDER_HEIGHT / 2 + SUPPORT_SURFACE_HEIGHT;

		// Orientation
		start_pose.orientation.x = 0;
		start_pose.orientation.y = 0;
		start_pose.orientation.z = 0;
		start_pose.orientation.w = 1;

		return start_pose;
	}

	geometry_msgs::Pose createGoalPose() {
		geometry_msgs::Pose goal_pose;

		// Position
		goal_pose.position.x = 0.0;
		goal_pose.position.y = -0.2;
		goal_pose.position.z = CYLINDER_HEIGHT / 2 + SUPPORT_SURFACE_HEIGHT;

		// Orientation
		goal_pose.orientation.x = 0;
		goal_pose.orientation.y = 0;
		goal_pose.orientation.z = 0;
		goal_pose.orientation.w = 1;

		return goal_pose;
	}

	bool move_to_neutral_position() {

		ROS_INFO_STREAM_NAMED("uibk_pick_place", "Moving arm into neutral position =============================== \n");
		double jnt_values[] = { 0, 0, 0, 0, 0, 0, 0 };
		vector<double> values;
		values.insert(values.begin(), jnt_values, jnt_values + 7);
		bool target_reached = false;

		while(!target_reached) {
			ROS_INFO_NAMED("uibk_pick_place", "Trying to execute motion...");
			move_group_->setJointValueTarget(values);
			target_reached = move_group_->move();
		}

		return true;
	}

	bool pick(const geometry_msgs::Pose& block_pose, std::string name) {
		ROS_WARN_STREAM_NAMED("", "picking object "<< name);

		std::vector<manipulation_msgs::Grasp> grasps;

		// Pick grasp
		grasp_generator_->generateGrasps(block_pose, grasp_data_, grasps);

		// Prevent collision with table
		move_group_->setSupportSurfaceName(SUPPORT_SURFACE_NAME);

		return move_group_->pick(name, grasps);
	}

	bool place(const geometry_msgs::Pose& goal_pose, std::string name) {
		ROS_WARN_STREAM_NAMED("pick_place", "Placing "<< name);

		std::vector<manipulation_msgs::PlaceLocation> place_locations;
		//std::vector<manipulation_msgs::Grasp> grasps;

		// Re-usable datastruct
		geometry_msgs::PoseStamped pose_stamped;
		pose_stamped.header.frame_id = BASE_LINK;
		pose_stamped.header.stamp = ros::Time::now();

		// Create 360 degrees of place location rotated around a center
		for (double angle = 0; angle < 2 * M_PI; angle += M_PI / 4) {
			pose_stamped.pose = goal_pose;

			// Orientation
			Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitZ()));
			pose_stamped.pose.orientation.x = quat.x();
			pose_stamped.pose.orientation.y = quat.y();
			pose_stamped.pose.orientation.z = quat.z();
			pose_stamped.pose.orientation.w = quat.w();

			// Create new place location
			manipulation_msgs::PlaceLocation place_loc;

			place_loc.place_pose = pose_stamped;

			//ROS_INFO_STREAM_NAMED("temp","pose:\n" << place_loc.place_pose);
			visual_tools_->publishCylinder(place_loc.place_pose.pose, CYLINDER_HEIGHT, CYLINDER_RADIUS, true);

			// Approach
			manipulation_msgs::GripperTranslation gripper_approach;
			gripper_approach.direction.header.stamp = ros::Time::now();
			gripper_approach.desired_distance = grasp_data_.approach_retreat_desired_dist_; // The distance the origin of a robot link needs to travel
			gripper_approach.min_distance = grasp_data_.approach_retreat_min_dist_; // half of the desired? Untested.
			gripper_approach.direction.header.frame_id = grasp_data_.ee_parent_link_;
			gripper_approach.direction.vector.x = 1;
			gripper_approach.direction.vector.y = 0;
			gripper_approach.direction.vector.z = 0; // Approach direction (negative z axis)  // TODO: document this assumption
			place_loc.approach = gripper_approach;

			// Retreat
			manipulation_msgs::GripperTranslation gripper_retreat;
			gripper_retreat.direction.header.stamp = ros::Time::now();
			gripper_retreat.desired_distance = grasp_data_.approach_retreat_desired_dist_; // The distance the origin of a robot link needs to travel
			gripper_retreat.min_distance = grasp_data_.approach_retreat_min_dist_; // half of the desired? Untested.
			gripper_retreat.direction.header.frame_id = grasp_data_.ee_parent_link_;
			gripper_retreat.direction.vector.x = 0;
			gripper_retreat.direction.vector.y = 0;
			gripper_retreat.direction.vector.z = -1; // Retreat direction (pos z axis)
			place_loc.retreat = gripper_retreat;

			// Post place posture - use same as pre-grasp posture (the OPEN command)
			place_loc.post_place_posture = grasp_data_.pre_grasp_posture_;

			place_locations.push_back(place_loc);
		}

		// Prevent collision with table
		move_group_->setSupportSurfaceName(SUPPORT_SURFACE_NAME);

		moveit_msgs::OrientationConstraint oc;
		oc.header.frame_id = BASE_LINK;
		oc.link_name = EE_PARENT_LINK;

		oc.orientation.x = 0;
		oc.orientation.y = 0;
		oc.orientation.z = 0;
		oc.orientation.w = 1;

		oc.absolute_x_axis_tolerance = 0.3;
		oc.absolute_y_axis_tolerance = 0.3;
		oc.absolute_z_axis_tolerance = 0.3;

		oc.weight = 1;

		moveit_msgs::Constraints constraints;
		constraints.orientation_constraints.push_back(oc);

		// move_group_->setPathConstraints(constraints);
		move_group_->setPlannerId("RRTConnectkConfigDefault");

		return move_group_->place(name, place_locations);
	}

	bool promptUser() {
		// Make sure ROS is still with us
		if (!ros::ok())
			return false;

		if (auto_reset_) {
			ROS_INFO_STREAM_NAMED("pick_place", "Auto-retrying in " << auto_reset_sec_ << " seconds");
			ros::Duration(auto_reset_sec_).sleep();
		} else {
			ROS_INFO_STREAM_NAMED("pick_place", "Retry? (y/n)");
			char input; // used for prompting yes/no
			cin >> input;
			if (input == 'n')
				return false;
		}
		return true;
	}

}
;

}

int main(int argc, char **argv) {
	ros::init(argc, argv, "test_pick_place");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	uibk_pick_place::TestPickPlace t;
	t.start();

	return 0;
}
