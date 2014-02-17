/*
 * visualization_tools.cpp
 *
 *  Created on: Nov 8, 2013
 *      Author: martin
 */

#include <uibk_pick_place/visualization_tools.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <shape_tools/solid_primitive_dims.h>

using namespace std;
using namespace planning_scene_monitor;

namespace uibk_pick_place {

VisualizationTools::VisualizationTools(string base_link, PlanningSceneMonitorPtr planning_scene_monitor, string marker_topic):
				planning_scene_monitor_(planning_scene_monitor),
				muted_(false)
{
	// Pass to next contructor
	VisualizationTools(base_link, marker_topic);
}

VisualizationTools::VisualizationTools(string base_link, string marker_topic):
		nh_(),
	    marker_topic_(marker_topic),
	    ee_group_name_("unknown"),
	    planning_group_name_("unknown"),
	    base_link_(base_link),
	    marker_lifetime_(ros::Duration(30.0)),
	    muted_(false) {

	// Rviz Visualizations
	pub_rviz_marker_ = nh_.advertise<visualization_msgs::Marker>(marker_topic_, 10);
	ROS_DEBUG_STREAM_NAMED("viz_tools","Visualizing rviz markers on topic " << marker_topic_);

	// Collision object creator
	pub_collision_obj_ = nh_.advertise<moveit_msgs::CollisionObject>(COLLISION_TOPIC, 10);
	ROS_DEBUG_STREAM_NAMED("viz_tools","Publishing collision objects on topic " << COLLISION_TOPIC);

	// Collision object attacher
	pub_attach_collision_obj_ = nh_.advertise<moveit_msgs::AttachedCollisionObject>(ATTACHED_COLLISION_TOPIC, 10);
	ROS_DEBUG_STREAM_NAMED("viz_tools","Publishing attached collision objects on topic " << ATTACHED_COLLISION_TOPIC);

	// Trajectory paths
	pub_display_path_ = nh_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 10, true);

	// Cache the reusable markers
	loadRvizMarkers();

	// Wait
	ros::spinOnce();
	ros::Duration(0.5).sleep();
}

VisualizationTools::~VisualizationTools() {
}

void VisualizationTools::loadRvizMarkers() {
	// Load arrow ----------------------------------------------------

	arrow_marker_.header.frame_id = base_link_;
	// Set the namespace and id for this marker.  This serves to create a unique ID
	arrow_marker_.ns = "Arrow";
	// Set the marker type.
	arrow_marker_.type = visualization_msgs::Marker::ARROW;
	// Set the marker action.  Options are ADD and DELETE
	arrow_marker_.action = visualization_msgs::Marker::ADD;
	// Size
	arrow_marker_.scale.x = 0.05; //0.025; // arrow width - but i would call this the length
	arrow_marker_.scale.y = 0.005; // arrow height
	arrow_marker_.scale.z = 0.005; // arrow length
	// Lifetime
	arrow_marker_.lifetime = marker_lifetime_;

	// Load Block ----------------------------------------------------
	block_marker_.header.frame_id = base_link_;
	// Set the namespace and id for this marker.  This serves to create a unique ID
	block_marker_.ns = "Block";
	// Set the marker action.  Options are ADD and DELETE
	block_marker_.action = visualization_msgs::Marker::ADD;
	// Set the marker type.
	block_marker_.type = visualization_msgs::Marker::CUBE;
	// Lifetime
	block_marker_.lifetime = marker_lifetime_;

	// Load Cylinder ----------------------------------------------------
	cylinder_marker_.header.frame_id = base_link_;
	// Set the namespace and id for this marker.  This serves to create a unique ID
	cylinder_marker_.ns = "Cylinder";
	// Set the marker action.  Options are ADD and DELETE
	cylinder_marker_.action = visualization_msgs::Marker::ADD;
	// Set the marker type.
	cylinder_marker_.type = visualization_msgs::Marker::CYLINDER;
	// Lifetime
	cylinder_marker_.lifetime = marker_lifetime_;

	// Load Sphere -------------------------------------------------
	sphere_marker_.header.frame_id = base_link_;
	// Set the namespace and id for this marker.  This serves to create a unique ID
	sphere_marker_.ns = "Sphere";
	// Set the marker type.
	sphere_marker_.type = visualization_msgs::Marker::SPHERE_LIST;
	// Set the marker action.  Options are ADD and DELETE
	sphere_marker_.action = visualization_msgs::Marker::ADD;
	// Marker group position and orientation
	sphere_marker_.pose.position.x = 0;
	sphere_marker_.pose.position.y = 0;
	sphere_marker_.pose.position.z = 0;
	sphere_marker_.pose.orientation.x = 0.0;
	sphere_marker_.pose.orientation.y = 0.0;
	sphere_marker_.pose.orientation.z = 0.0;
	sphere_marker_.pose.orientation.w = 1.0;
	// Sphere size
	sphere_marker_.scale.x = 0.01;
	sphere_marker_.scale.y = 0.01;
	sphere_marker_.scale.z = 0.01;
	// Color
	sphere_marker_.color = getColor( BLUE );
	// Create a sphere point
	geometry_msgs::Point point_a;
	// Add the point pair to the line message
	sphere_marker_.points.push_back( point_a );
	sphere_marker_.colors.push_back( getColor( BLUE ) );

	// Load Text ----------------------------------------------------
	text_marker_.header.frame_id = base_link_;
	// Set the namespace and id for this marker.  This serves to create a unique ID
	text_marker_.ns = "Text";
	// Set the marker action.  Options are ADD and DELETE
	text_marker_.action = visualization_msgs::Marker::ADD;
	// Set the marker type.
	text_marker_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	// Lifetime
	text_marker_.lifetime = marker_lifetime_;
}

void VisualizationTools::attachCO(const std::string& name) {
	// Clean up old attached collision object
	moveit_msgs::AttachedCollisionObject aco;
	aco.object.header.stamp = ros::Time::now();
	aco.object.header.frame_id = base_link_;

	aco.object.id = name;
	aco.object.operation = moveit_msgs::CollisionObject::ADD;

	// Link to attach the object to
	aco.link_name = ee_parent_link_;

	ros::WallDuration(0.1).sleep();
	pub_attach_collision_obj_.publish(aco);
	ros::WallDuration(0.1).sleep();
	pub_attach_collision_obj_.publish(aco);
}

void VisualizationTools::cleanupCO(const std::string& name) {
	// Clean up old collision objects
	moveit_msgs::CollisionObject co;
	co.header.stamp = ros::Time::now();
	co.header.frame_id = base_link_;
	co.id = name;
	co.operation = moveit_msgs::CollisionObject::REMOVE;
	ros::WallDuration(0.1).sleep();
	pub_collision_obj_.publish(co);
	ros::WallDuration(0.1).sleep();
	pub_collision_obj_.publish(co);
}

void VisualizationTools::cleanupACO(const std::string& name) {
	// Clean up old attached collision object
	moveit_msgs::AttachedCollisionObject aco;
	aco.object.header.stamp = ros::Time::now();
	aco.object.header.frame_id = base_link_;

	//aco.object.id = name;
	aco.object.operation = moveit_msgs::CollisionObject::REMOVE;

	aco.link_name = ee_parent_link_;

	ros::WallDuration(0.1).sleep();
	pub_attach_collision_obj_.publish(aco);
	ros::WallDuration(0.1).sleep();
	pub_attach_collision_obj_.publish(aco);
}

bool VisualizationTools::loadPlanningSceneMonitor() {
	// ---------------------------------------------------------------------------------------------
	// Create planning scene monitor
	planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor(ROBOT_DESCRIPTION));

	ros::spinOnce();
	ros::Duration(0.5).sleep();
	ros::spinOnce();

	if (planning_scene_monitor_->getPlanningScene()) {
		//planning_scene_monitor_->startWorldGeometryMonitor();
		//planning_scene_monitor_->startSceneMonitor("/move_group/monitored_planning_scene");
		//planning_scene_monitor_->startStateMonitor("/joint_states", "/attached_collision_object");
		//planning_scene_monitor_->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
		//  "dave_planning_scene");
	} else {
		ROS_FATAL_STREAM_NAMED("rviz_tools", "Planning scene not configured");
		return false;
	}

	ros::spinOnce();
	ros::Duration(0.5).sleep();
	ros::spinOnce();

	return true;
}

bool VisualizationTools::loadEEMarker() {
	// Check if we have already loaded the EE markers
	if (ee_marker_array_.markers.size() > 0) // already loaded
		return true;

	// -----------------------------------------------------------------------------------------------
	// Get end effector group

	// Create color to use for EE markers
	std_msgs::ColorRGBA marker_color = getColor(GREY);

	// Get robot model
	robot_model::RobotModelConstPtr robot_model = getPlanningSceneMonitor()->getRobotModel();
	// Get joint state group
	//robot_state::JointStateGroup* joint_state_group = robot_state.getJointStateGroup(ee_group_name_);
	const robot_model::JointModelGroup* joint_model_group = robot_model->getJointModelGroup(ee_group_name_);
	if (joint_model_group == NULL) // make sure EE_GROUP exists
	{
		ROS_ERROR_STREAM_NAMED("viz_tools", "Unable to find joint model group " << ee_group_name_);
		return false;
	}

	// Get link names that are in end effector
	const std::vector<std::string> &ee_link_names = joint_model_group->getLinkModelNames();

	ROS_DEBUG_STREAM_NAMED("viz_tools", "Number of links in group " << ee_group_name_ << ": " << ee_link_names.size());
	//std::copy(ee_link_names.begin(), ee_link_names.end(), std::ostream_iterator<std::string>(std::cout, "\n"));

	// Robot Interaction - finds the end effector associated with a planning group
	//robot_interaction::RobotInteraction robot_interaction( getPlanningSceneMonitor()->getRobotModel() );
	robot_interaction::RobotInteraction robot_interaction(robot_model);

	// Decide active end effectors
	robot_interaction.decideActiveEndEffectors(planning_group_name_);

	// Get active EE
	std::vector<robot_interaction::RobotInteraction::EndEffector> active_eef =
			robot_interaction.getActiveEndEffectors();

	ROS_DEBUG_STREAM_NAMED("viz_tools", "Number of active end effectors: " << active_eef.size());
	if (!active_eef.size()) {
		ROS_ERROR_STREAM_NAMED("viz_tools",
				"No active end effectors found! Make sure kinematics.yaml is loaded in this node's namespace!");
		return false;
	}

	// Just choose the first end effector \todo better logic?
	robot_interaction::RobotInteraction::EndEffector eef = active_eef[0];

	// -----------------------------------------------------------------------------------------------
	// Get EE link markers for Rviz
	robot_state::RobotState robot_state = getPlanningSceneMonitor()->getPlanningScene()->getCurrentState();
	//    robot_state.updateTransforms();

	/*ROS_ERROR_STREAM_NAMED("temp","before printing");
	 robot_state.printStateInfo();
	 robot_state.printTransforms();
	 */

	robot_state.getRobotMarkers(ee_marker_array_, ee_link_names, marker_color, eef.eef_group, ros::Duration());
	ROS_DEBUG_STREAM_NAMED("viz_tools", "Number of rviz markers in end effector: " << ee_marker_array_.markers.size());

	// Change pose from Eigen to TF
	try {
		ee_parent_link_ = eef.parent_link; // save the name of the link for later use
		tf::poseEigenToTF(robot_state.getFrameTransform(eef.parent_link), tf_root_to_link_);
		//tf::poseEigenToTF(robot_state.getLinkState(eef.parent_link)->getGlobalLinkTransform(), tf_root_to_link_);
	} catch (...) {
		ROS_ERROR_STREAM_NAMED("viz_tools", "Didn't find link state for " << ee_parent_link_);
	}

	// Copy original marker poses to a vector
	for (std::size_t i = 0; i < ee_marker_array_.markers.size(); ++i) {
		marker_poses_.push_back(ee_marker_array_.markers[i].pose);
	}

	return true;
}

bool VisualizationTools::publishEEMarkers(const geometry_msgs::Pose& pose, const rviz_colors& color,
		const std::string& ns)
{
	if (muted_)
		return true;

	// Load EE Markers
	if (!loadEEMarker()) {
		ROS_ERROR_STREAM_NAMED("viz_tools", "Unable to publish EE marker");
		return false;
	}

	// -----------------------------------------------------------------------------------------------
	// Change the end effector pose to frame of reference of this custom end effector

	// Convert to Eigen
	Eigen::Affine3d ee_pose_eigen;
	Eigen::Affine3d eef_conversion_pose;
	tf::poseMsgToEigen(pose, ee_pose_eigen);
	tf::poseMsgToEigen(grasp_pose_to_eef_pose_, eef_conversion_pose);

	// Transform the grasp pose
	ee_pose_eigen = ee_pose_eigen * eef_conversion_pose;

	// Convert back to message
	geometry_msgs::Pose ee_pose;     // Non const version
	tf::poseEigenToMsg(ee_pose_eigen, ee_pose);

	// -----------------------------------------------------------------------------------------------
	// Process each link of the end effector
	for (std::size_t i = 0; i < ee_marker_array_.markers.size(); ++i) {
		// Make sure ROS is still spinning
		if (!ros::ok())
			break;

		// Header
		ee_marker_array_.markers[i].header.frame_id = base_link_;
		ee_marker_array_.markers[i].header.stamp = ros::Time::now();

		// Namespace
		ee_marker_array_.markers[i].ns = ns;

		// Lifetime
		ee_marker_array_.markers[i].lifetime = marker_lifetime_;

		// Color
		ee_marker_array_.markers[i].color = getColor(color);

		// Options for meshes
		if (ee_marker_array_.markers[i].type == visualization_msgs::Marker::MESH_RESOURCE) {
			ee_marker_array_.markers[i].mesh_use_embedded_materials = true;
		}

		// -----------------------------------------------------------------------------------------------
		// Do some math for the offset
		// pose             - our generated grasp
		// markers[i].pose        - an ee link's pose relative to the whole end effector
		// REMOVED grasp_pose_to_eef_pose_ - the offset from the grasp pose to eef_pose - probably nothing
		tf::Pose tf_root_to_marker;
		tf::Pose tf_root_to_mesh;
		tf::Pose tf_pose_to_eef;

		// Simple conversion from geometry_msgs::Pose to tf::Pose
		tf::poseMsgToTF(pose, tf_root_to_marker);
		tf::poseMsgToTF(marker_poses_[i], tf_root_to_mesh);
		// tf::poseMsgToTF(grasp_pose_to_eef_pose_, tf_pose_to_eef); // \todo REMOVE

		// Conversions
		tf::Pose tf_eef_to_mesh = tf_root_to_link_.inverse() * tf_root_to_mesh;
		// REMOVED tf::Pose tf_marker_to_mesh = tf_pose_to_eef * tf_eef_to_mesh;
		//tf::Pose tf_root_to_mesh_new = tf_root_to_marker * tf_marker_to_mesh;
		tf::Pose tf_root_to_mesh_new = tf_root_to_marker * tf_eef_to_mesh;
		tf::poseTFToMsg(tf_root_to_mesh_new, ee_marker_array_.markers[i].pose);
		// -----------------------------------------------------------------------------------------------

		//ROS_INFO_STREAM("Marker " << i << ":\n" << ee_marker_array_.markers[i]);

		pub_rviz_marker_.publish(ee_marker_array_.markers[i]);
		ros::spinOnce();
	}

	return true;
}

/**
 * Show a small arrow in given color at given pose in rviz
 * @param pose
 * @param color
 * @return
 */
bool VisualizationTools::publishArrow(const geometry_msgs::Pose& pose, const rviz_colors color) {

	if (muted_)
		return true;

	// Set the frame ID and timestamp.  See the TF tutorials for information on these.
	arrow_marker_.header.stamp = ros::Time::now();

	static int id = 0;
	arrow_marker_.id = ++id;

	arrow_marker_.pose = pose;

	arrow_marker_.color = getColor(color);

	pub_rviz_marker_.publish(arrow_marker_);
	ros::spinOnce();

	return true;
}

bool VisualizationTools::publishBlock(const geometry_msgs::Pose& pose, const double& size, const bool isRed) {
	if(muted_)
	    return true;

	// Set the timestamp
	block_marker_.header.stamp = ros::Time::now();

	static int id = 0;
	block_marker_.id = ++id;

	// Set the pose
	block_marker_.pose = pose;

	// Set marker size
	block_marker_.scale.x = size;
	block_marker_.scale.y = size;
	block_marker_.scale.z = size;

	// Set marker color
	if(isRed)
	{
		block_marker_.color = getColor( RED );
	}
	else
	{
		block_marker_.color = getColor( GREEN );
	}

	pub_rviz_marker_.publish(block_marker_);

	return true;
}

bool VisualizationTools::publishCylinder(const geometry_msgs::Pose& pose, const double& height,
		const double& radius, const bool isRed) {

	if(muted_)
		    return true;

	// Set the timestamp
	cylinder_marker_.header.stamp = ros::Time::now();

	static int id = 0;
	cylinder_marker_.id = ++id;

	// Set the pose
	cylinder_marker_.pose = pose;

	// Set marker size
	cylinder_marker_.scale.x = 2 * radius;
	cylinder_marker_.scale.y = 2 * radius;
	cylinder_marker_.scale.z = height;

	// Set marker color
	if(isRed)
	{
		cylinder_marker_.color = getColor( RED );
	}
	else
	{
		cylinder_marker_.color = getColor( GREEN );
	}

	pub_rviz_marker_.publish(cylinder_marker_);
	//ros::Duration(0.05).sleep(); // Sleep to prevent markers from being 'skipped' in rviz

	return true;
}

bool VisualizationTools::publishText(const geometry_msgs::Pose& pose, const string& text, const rviz_colors& color) {

	if(muted_)
	    return true;

	text_marker_.id = 0;

	text_marker_.header.stamp = ros::Time::now();
	text_marker_.text = text;
	text_marker_.pose = pose;
	text_marker_.color = getColor(color);
	text_marker_.scale.z = 0.01;    // only z is required (size of an "A")

	pub_rviz_marker_.publish( text_marker_ );

	return true;
}

void VisualizationTools::publishCollisionBlock(const geometry_msgs::Pose& pose, const string& name, double size) {
	publishCollisionBlock(pose, size, size, size, name);
}

void VisualizationTools::publishCollisionBlock(const geometry_msgs::Pose& pose, double width, double height, double depth, const string& name) {
	moveit_msgs::CollisionObject collision_obj;

	collision_obj.header.stamp = ros::Time::now();
	collision_obj.header.frame_id = base_link_;
	collision_obj.id = name;
	collision_obj.operation = moveit_msgs::CollisionObject::ADD;
	collision_obj.primitives.resize(1);
	collision_obj.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
	collision_obj.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);

	// Size
	collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = depth;
	collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = width;
	collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = height;

	collision_obj.primitive_poses.resize(1);
	collision_obj.primitive_poses[0] = pose;

	pub_collision_obj_.publish(collision_obj);

	ROS_DEBUG_STREAM_NAMED("uibk_pick_place","Published collision object " << name);
}

void VisualizationTools::publishCollisionCylinder(const geometry_msgs::Pose& pose, const string& name, double height, double radius) {

	moveit_msgs::CollisionObject collision_obj;

	collision_obj.header.stamp = ros::Time::now();
	collision_obj.header.frame_id = base_link_;
	collision_obj.id = name;
	collision_obj.operation = moveit_msgs::CollisionObject::ADD;
	collision_obj.primitives.resize(1);
	collision_obj.primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
	collision_obj.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::CYLINDER>::value);
	collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = height;
	collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = radius;
	collision_obj.primitive_poses.resize(1);
	collision_obj.primitive_poses[0] = pose;

	pub_collision_obj_.publish(collision_obj);

	ROS_DEBUG_STREAM_NAMED("uibk_pick_place", "Published collision object " << name);
}

bool VisualizationTools::publishTrajectoryPath(const moveit_msgs::RobotTrajectory& trajectory_msg, bool waitTrajectory) {

	// Create the message
	moveit_msgs::DisplayTrajectory rviz_display;
	//rviz_display.model_id = getPlanningSceneMonitor()->getPlanningScene()->getRobotModel()->getName();

	rviz_display.trajectory.resize(1);
	rviz_display.trajectory[0] = trajectory_msg;

	// Publish message
	pub_display_path_.publish(rviz_display);

	ros::spinOnce();
	ros::Duration(0.1).sleep();

	// Wait the duration of the trajectory
	if (waitTrajectory) {
		ros::Duration wait_sec = trajectory_msg.joint_trajectory.points.back().time_from_start * 4;
		ROS_INFO_STREAM_NAMED("uibk_pick_place", "Waiting for trajectory animation " << wait_sec.toSec() << " seconds");
		wait_sec.sleep();
	}

	return true;
}

std_msgs::ColorRGBA VisualizationTools::getColor(const rviz_colors& color) {
	std_msgs::ColorRGBA result;
	result.a = 0.8;
	switch(color)
	{
		case RED:
			result.r = 0.8;
			result.g = 0.1;
			result.b = 0.1;
			break;
		case GREEN:
			result.r = 0.1;
			result.g = 0.8;
			result.b = 0.1;
			break;
		case GREY:
			result.r = 0.9;
			result.g = 0.9;
			result.b = 0.9;
			break;
		case WHITE:
			result.r = 1.0;
			result.g = 1.0;
			result.b = 1.0;
			break;
		case ORANGE:
			result.r = 1.0;
			result.g = 0.5;
			result.b = 0.0;
			break;
		case BLUE:
		default:
			result.r = 0.1;
			result.g = 0.1;
			result.b = 0.8;
	  }

	  return result;
}

} // namespace
