/* Author: Emanuele Sansebastiano
   Desc: Class to encapsulate some side functions generated for the path planning part
         Amazon Robotics Challenge - Universidad Jaume I (Spain) Competitor
*/

#ifndef MOVEIT_SIDE_PKG_SIDE_FUNCTIONS_H
#define MOVEIT_SIDE_PKG_SIDE_FUNCTIONS_H

// ROS
#include <ros/ros.h>

// C++
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <boost/math/constants/constants.hpp>

// Moveit! libraries
#include <moveit_msgs/OrientationConstraint.h>

#include <moveit/move_group_interface/move_group.h>
/*
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
*/

namespace moveit_side_functions
{
  //brief: Convert radiant to degree
  double rad2deg(double rad);

  //brief: Convert degree to radiant
  double deg2rad(double deg);

  //brief: Absolute function to convert double values
  double abs_f(double val);

  //brief: Round a value to the closest integer
  //       examples: 2.4 --> 2; 2.7 --> 3; 2.0 --> 2; 2.5 --> 3;
  int round_f(double val);

  //brief: Function to cluster to vector in one vector
  std::vector<double> vector_two_cluster(std::vector<double> left, std::vector<double> right);

  //brief: Make a Vector3 data in just one line
  geometry_msgs::Vector3 makeVector3(double x, double y, double z);

  //brief: Make a Quaternion data in just one line
  geometry_msgs::Quaternion makeQuat(double w, double x, double y, double z);

  //brief: Function to make a Pose message from quaternions
  geometry_msgs::Pose makePose(geometry_msgs::Quaternion orientation, geometry_msgs::Vector3 XYZ_location);
  //brief: Function to make a Pose message from eulerian angles
  geometry_msgs::Pose makePose(geometry_msgs::Vector3 RPY_orientation, geometry_msgs::Vector3 XYZ_location);

  //brief: Convert a Pose data to a PoseStamped data
  geometry_msgs::PoseStamped Pose2PoseStamped( geometry_msgs::Pose old_pose);

  //brief: Convert a PoseStamped data to a Pose data
  geometry_msgs::Pose PoseStamped2Pose( geometry_msgs::PoseStamped old_pose_s);

  //brief: Function to convert RPY to quaternion angles
  //       If RPY has no radiant unit change RPY_rad to false
  //       If you do not want to arrange the angle in to interval (-pi ; pi] change turn_corr to false
  geometry_msgs::Quaternion RPY2Quat(geometry_msgs::Vector3 RPY, bool RPY_rad = true, bool turn_corr = true);

  //brief: This function return the sum of the absolute difference of every term of two quaternions
  double Quat_diff(geometry_msgs::Quaternion a, geometry_msgs::Quaternion b);

// End namespace "moveit_side_functions"
}

namespace moveit_basics_bax
{
//brief: Predefined vertical orientation (180.0; 0.0; 0.0)
  geometry_msgs::Vector3 vertical_orientation_x();

  //brief: Predefined vertical orientation (0.0; 180.0; 0.0)
  geometry_msgs::Vector3 vertical_orientation_y();

  //brief: Function to define the location over the current gripper location
  //       Get the current z angle using "move_group.h --> getCurrentRPY(std::string ee_name)
  //       This function choose the closest final orientation among the two possible verticals
  geometry_msgs::Pose up_position(geometry_msgs::Pose ee_curr_pos, double curr_z = 0.0, double distance = 0.2);

  //brief: Function to define a orientation constraint parameter from quaternions
  moveit_msgs::OrientationConstraint orient_constr_definition(geometry_msgs::Quaternion orientation, std::string link_name,
  		  float toll_x = 0.1, float toll_y = 0.1, float toll_z = 0.1, float weight = 1.0, std::string header_frame_id = "world");
  //brief: Function to define a orientation constraint parameter from quaternions
  moveit_msgs::OrientationConstraint orient_constr_definition(geometry_msgs::Vector3 RPY, std::string link_name,
   		  float toll_x = 0.1, float toll_y = 0.1, float toll_z = 0.1, float weight = 1.0, std::string header_frame_id = "world");

  //brief: Function to return the list of the available planners
  std::vector<std::string> getOmplPlannerList(void);

// End namespace "moveit_basics_bax"
}

namespace obj_functions
{
  ////// ROBOT FUNCTIONS //////
  // The object must be instantiated in the executable file using:
  // moveit::planning_interface::MoveGroup obj_name(std::string part_name)
  // part_name could be: "right_arm", "left_arm", or "both_arms"

  //brief: This function returns the list of the possible group names
  std::vector<std::string> GroupNameAvailable(void);

  //brief: Function to get the link names
  std::vector<std::string> getLinkNames(moveit::planning_interface::MoveGroup& obj);

  //brief: Function to get the joint names
  std::vector<std::string> getJointNames(moveit::planning_interface::MoveGroup& obj);

  //brief: Function to change the planner ID among the ones available
  bool setPlanner(moveit::planning_interface::MoveGroup& obj, std::string new_planner);

  //brief: Function to set the pose of the end-effector
  //       !Do not forget there is a function to generate a Pose msg (moveit_side_functions::makePose)
  bool setEePoseTarget(moveit::planning_interface::MoveGroup& obj,
		  geometry_msgs::Pose pose, std::string left_right = "");

  //brief: Function to impose the joint value target directly
  //       Change "left_right_both" just in case both arms are controlled, but you want to move just one of them
  //       Do not forget to cluster the joint position with the function: 'moveit_side_functions::vector_two_cluster'.
  //       The values in the vector must go from the shoulder to the wrist.
  bool setJointValuesTarget(moveit::planning_interface::MoveGroup& obj,
		  std::vector<double> vector, std::string r_l_single = "");

  void move_f(moveit::planning_interface::MoveGroup& obj);


  ////// ENVIRONMENT FUNCTIONS //////


// End namespace "obj_functions"
}


#endif /* MOVEIT_SIDE_PKG_SIDE_FUNCTIONS_H */

/*
 * Main library for MoveIt!: move_group.h
 * Directory of move_group.h: ~/moveit/moveit_ros/planning_interface/move_group_interface/include/moveit/move_group_interface
 *
 * Object Constructor: MoveGroup
 * Object Destructor: ~MoveGroup
 *
 * Interesting functions included in this object definition:
 * - getName
 * - getNameTarget
 * - getRobotName
 * - getNodeHandle
 * - getJointName
 * - getLinkName
 * - getActiveJoints
 * - getJoints
 * - getDefaultPlannerId
 * - getCurrentJointValues
 * - getCurrentState
 * - getCurrentPose
 * - getCurrentRPY
 *
 * - setPlannerId
 * - setPlanningTime
 * - setNumPlanningAttempts
 * - setMaxVelocityScalingFactor
 * - setMaxAccelerationScalingFactor
 * - setGoal
 * - setWorkSpace
 * - setStartState
 * - setStartStateToCurrentState
 * - setSupportSurfaceName
 * - setJointValueTarget
 * - setApproximateJointValueTarget
 * - setPositionTarget
 * - setRPYTarget
 * - setOrientationTarget
 * - setPoseTarget
 * - setPathConstraints
 * - setGoalPositionTolerance
 * - setGoalOrientationTolerance
 *
 * - clearPoseTarget
 * - clearPathConstraints
 *
 * - attachObject
 * - detachObject
 *
 * - computeCartesianPath
 * - Move
 * - Plan
 * - Execute
 * - asyncMove
 * - asyncExecute
 * - Stop
 * - Pick
 * - Place
 *
 */

