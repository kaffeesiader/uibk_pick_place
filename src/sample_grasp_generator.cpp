/*
 * sample_grasp_generator.cpp
 *
 *  Created on: Nov 8, 2013
 *      Author: martin
 */

#include <uibk_pick_place/sample_grasp_generator.h>

using namespace std;

namespace uibk_pick_place {

SampleGraspGenerator::SampleGraspGenerator(VisualizationToolsPtr visualization_tools):
	rviz_tools_(visualization_tools)
{}

SampleGraspGenerator::~SampleGraspGenerator() {
}

bool SampleGraspGenerator::generateAxisGrasps(vector<manipulation_msgs::Grasp>& possible_grasps, const RobotGraspData& grasp_data)
{

	// ---------------------------------------------------------------------------------------------
	// Grasp parameters

	// Create re-usable approach motion
	manipulation_msgs::GripperTranslation gripper_approach;
	gripper_approach.direction.header.stamp = ros::Time::now();
	gripper_approach.desired_distance = grasp_data.approach_retreat_desired_dist_; // The distance the origin of a robot link needs to travel
	gripper_approach.min_distance = grasp_data.approach_retreat_min_dist_; // half of the desired? Untested.

	// Create re-usable retreat motion
	manipulation_msgs::GripperTranslation gripper_retreat;
	gripper_retreat.direction.header.stamp = ros::Time::now();
	gripper_retreat.desired_distance = grasp_data.approach_retreat_desired_dist_; // The distance the origin of a robot link needs to travel
	gripper_retreat.min_distance = grasp_data.approach_retreat_min_dist_; // half of the desired? Untested.

	// Create re-usable blank pose
	geometry_msgs::PoseStamped grasp_pose_msg;
	grasp_pose_msg.header.stamp = ros::Time::now();
	grasp_pose_msg.header.frame_id = grasp_data.base_link_;

	// ---------------------------------------------------------------------------------------------
	// Variables needed for calculations
	double radius = grasp_data.grasp_depth_; //0.12
	double xb = 0;
	double yb = 0;
	double zb = 0;
	double theta = M_PI;

	/* Developer Note:
	 * Create angles 90 degrees around the chosen axis at given resolution
	 * We create the grasps in the reference frame of the object, then later convert it to the base link
	 */
	for (int i = 0; i <= grasp_data.angle_resolution_; ++i) {
		// Calculate grasp
		xb = radius * -sin(theta);
		yb = radius * cos(theta);


		Eigen::Affine3d grasp_pose;

		grasp_pose = Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitX()) *
					 Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitY()) *
					 Eigen::AngleAxisd(-0.5 * M_PI, Eigen::Vector3d::UnitZ());

		grasp_pose.translation() = Eigen::Vector3d(xb, yb, zb);

		theta -= M_PI / 2 / grasp_data.angle_resolution_;

		// ---------------------------------------------------------------------------------------------
		// Create a Grasp message
		manipulation_msgs::Grasp new_grasp;

		// A name for this grasp
		static int grasp_id = 0;
		new_grasp.id = "Grasp" + boost::lexical_cast<std::string>(grasp_id);
		++grasp_id;

		// PreGrasp and Grasp Postures --------------------------------------------------------------------------

		// The internal posture of the hand for the pre-grasp only positions are used
		new_grasp.pre_grasp_posture = grasp_data.pre_grasp_posture_;

		// The internal posture of the hand for the grasp positions and efforts are used
		new_grasp.grasp_posture = grasp_data.grasp_posture_;

		// Grasp ------------------------------------------------------------------------------------------------

		// Convert pose to global frame (base_link)
		tf::poseEigenToMsg(global_transform_ * grasp_pose, grasp_pose_msg.pose);

		// TEMP
		//rviz_tools_->publishArrow(grasp_pose_msg.pose);

		// The position of the end-effector for the grasp relative to a reference frame (that is always specified elsewhere, not in this message)
		new_grasp.grasp_pose = grasp_pose_msg;

		// Other ------------------------------------------------------------------------------------------------

		// The estimated probability of success for this grasp, or some other measure of how "good" it is.
		new_grasp.grasp_quality = 1;

		// the maximum contact force to use while grasping (<=0 to disable)
		new_grasp.max_contact_force = 0;

		// an optional list of obstacles that we have semantic information about and that can be touched/pushed/moved in the course of grasping
		new_grasp.allowed_touch_objects.push_back("cylinder1");


		// Guessing -------------------------------------------------------------------------------------

		// Approach
		gripper_approach.direction.header.frame_id = grasp_data.ee_parent_link_;
		gripper_approach.direction.vector.x = 0;
		gripper_approach.direction.vector.y = 0;
		gripper_approach.direction.vector.z = 1;
		new_grasp.approach = gripper_approach;

		// Retreat
		gripper_retreat.direction.header.frame_id = grasp_data.ee_parent_link_;
		gripper_retreat.direction.vector.x = -1;
		gripper_retreat.direction.vector.y = 0;
		gripper_retreat.direction.vector.z = 0;
		new_grasp.retreat = gripper_retreat;

		// Add to vector
		possible_grasps.push_back(new_grasp);
	}

	return true;
}

bool SampleGraspGenerator::generateGrasps(const geometry_msgs::Pose& pose,
		const RobotGraspData& grasp_data, vector<manipulation_msgs::Grasp>& possible_grasps)
{

	// ---------------------------------------------------------------------------------------------
	// Create a transform from the object's frame to /base_link
	tf::poseMsgToEigen(pose, global_transform_);

	// ---------------------------------------------------------------------------------------------
	// Calculate grasps
	generateAxisGrasps(possible_grasps, grasp_data);

	ROS_INFO_STREAM_NAMED("grasp", "Generated " << possible_grasps.size() << " grasps.");

	// Visualize results
	visualizeGrasps(possible_grasps, pose, grasp_data);

	return true;
}

void SampleGraspGenerator::visualizeGrasps(const vector<manipulation_msgs::Grasp>& possible_grasps,
		const geometry_msgs::Pose& pose, const RobotGraspData& grasp_data)
{

	if (rviz_tools_->isMuted()) {
		ROS_DEBUG_STREAM_NAMED("grasp", "Not visualizing grasps - muted.");
		return; // this function will only work if we have loaded the publishers
	}

	ROS_DEBUG_STREAM_NAMED("grasp", "Visualizing " << possible_grasps.size() << " grasps");

	//rviz_tools_->publishCylinder(pose, grasp_data.object_height, grasp_data.object_width / 2, false);

	for (vector<manipulation_msgs::Grasp>::const_iterator grasp_it = possible_grasps.begin();
			grasp_it < possible_grasps.end(); ++grasp_it)
	{
		rviz_tools_->publishArrow(grasp_it->grasp_pose.pose);
		//rviz_tools_->publishEEMarkers(grasp_it->grasp_pose.pose);

		ros::Duration(0.2).sleep();
	}
}

}

