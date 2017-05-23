/* Author: Emanuele Sansebastiano
   Desc: Library to encapsulate some side functions generated for the path planning part
         This library has been developed for two arm robots
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
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// Baxter libraries
#include <baxter_core_msgs/EndpointState.h>
#include <baxter_core_msgs/SEAJointState.h>

/*
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
*/

//Macro Concatenation
#define MAC_SUM(A, B) A # B

//////////////////////////////////////////////////////////////////////////////////////
// VALUES MODIFIABLE BY THE USER \\

// Common define values
#define	std_time					0.01
#define	stdAttempts4booleanFunc		5
#define exit_finction_time			0.5 //[sec]
//Moveit! values
#define att_exit					5 //maximum # moveit planner attempts
#define time_exit					6.0 //maximum time value moveit planner can use
//The following define structures could change according to the robot
#define	generic_str					""
//Baxter default values
//DO NOT MODIFY THESE VALUES, except if it is strickly necessary!
#define right_def					"right"
#define left_def					"left"
#define gripper_def					"_gripper"
#define	std_head_frame				"/world"
#define arm_group_name				"_arm"
#define both_arms_group_name		"both_arms"
//baxter topic defintion
#define joint_state					"/robot/joint_states"
#define base_robot_part				"/robot/limb/"
#define end_point					"/endpoint_state"
//baxter default values
#define baxter_def_joint_toll		0.0001		//[rad]
#define baxter_def_orient_toll		0.001		//[rad]
#define baxter_def_pos_toll 		0.0001		//[rad]
#define baxter_attempts				1
#define baxter_max_factor			1.0


//brief: Function defining the joints names in your Robot | user can change the joints names
std::vector<std::string> joint_names(void)
{
	//Just modify the following line if strickly necessary
	const std::string joints_names [] = {"_s0", "_s1", "_e0", "_e1", "_w0", "_w1", "_w2"};
	std::vector<std::string> j_n;
	j_n.resize(sizeof(joints_names)/sizeof(*joints_names));
	for(int i = 0; i < j_n.size(); i++)
		j_n[i] = joints_names[i];

	return j_n;
}

//////////////////////////////////////////////////////////////////////////////////////



namespace moveit_side_functions
{
  //brief: Standard attempts for boolean function before exiting (false)
  int _get_attempt_val(int val = stdAttempts4booleanFunc);

  //brief: Standard sleep time - default value [seconds]
  void standardSleep(double sleep_time = std_time);

  //brief: Convert radiant to degree
  double rad2deg(double rad);

  //brief: Convert degree to radiant
  double deg2rad(double deg);

  //brief: Absolute function to convert double values
  double abs_f(double val);

  //brief: Round a value to the closest integer
  //       examples: 2.4 --> 2; 2.7 --> 3; 2.0 --> 2; 2.5 --> 3;
  int round_f(double val);

  //brief: Function to shift decimal values and cut off the non-integer part
  int decimal_shift(double val2shift, int decimal_considered);

  //brief: Function to cluster to vector in one vector
  std::vector<double> vector_two_cluster(std::vector<double> left, std::vector<double> right);

  //brief: Make a Vector3 data in just one line
  geometry_msgs::Vector3 makeVector3(double x, double y, double z);

  //brief: Make a Quaternion data in just one line
  geometry_msgs::Quaternion makeQuat(double w, double x, double y, double z);

  //brief: Function to convert RPY to quaternion angles
  //       If RPY has no radiant unit change RPY_rad to false
  //       If you do not want to arrange the angle in to interval (-pi ; pi] change turn_corr to false
  geometry_msgs::Quaternion RPY2Quat(geometry_msgs::Vector3 RPY, bool RPY_rad = true, bool turn_corr = true);

  //brief: Function to make a Pose message from quaternions
  geometry_msgs::Pose makePose(geometry_msgs::Quaternion orientation, geometry_msgs::Vector3 XYZ_location);
  //brief: Function to make a Pose message from eulerian angles in degree
  geometry_msgs::Pose makePose(geometry_msgs::Vector3 RPY_orientation, geometry_msgs::Vector3 XYZ_location);

  //brief: Convert a Pose data to a PoseStamped data
  geometry_msgs::PoseStamped Pose2PoseStamped( geometry_msgs::Pose old_pose);

  //brief: Convert a PoseStamped data to a Pose data
  geometry_msgs::Pose PoseStamped2Pose( geometry_msgs::PoseStamped old_pose_s);

  //brief: This function return the sum of the absolute difference of every term of two quaternions
  double Quat_diff(geometry_msgs::Quaternion a, geometry_msgs::Quaternion b);

  //brief: This function compare two Pose
  //       'decimal_considered' is the number of decimals you want to consider - negative values mean 'all the available values'
  bool PoseEquivalence(geometry_msgs::Pose A, geometry_msgs::Pose B, int decimal_considered = -1);

  //brief: This function checks if a topic exist or not
  bool CheckTopicExistence(std::string topic2check);

// End namespace "moveit_side_functions"
}

namespace moveit_basics_functions
{
  //brief: Predefined vertical orientation (180.0; 0.0; 0.0)
  geometry_msgs::Vector3 vertical_orientation_x();

  //brief: Predefined vertical orientation (0.0; 180.0; 0.0)
  geometry_msgs::Vector3 vertical_orientation_y();

  //brief: Function to define the location over the current gripper location
  //       Get the current z angle using "move_group.h --> getCurrentRPY(std::string ee_name)
  //       This function choose the closest final orientation among the two possible verticals
  geometry_msgs::Pose up_position(geometry_msgs::Pose ee_curr_pos, double curr_z = 0.0, double distance = 0.2);

  //Comment: Other constraints could be set like 'PositionConstraints', but they are useless for this project.
  //         Check moveit_msgs/msg/Constraints.msg file to know more about them.
  //brief: Function to define a orientation constraint parameter from quaternions
  //       there is a function to know which is the planning frame "group.getPlanningFrame()" default = "/world"
  moveit_msgs::OrientationConstraint orient_constr_definition(geometry_msgs::Quaternion orientation, std::string link_name,
  		  float toll_x = 0.1, float toll_y = 0.1, float toll_z = 0.1, float weight = 1.0, std::string header_frame_id = std_head_frame);
  //brief: Function to define a orientation constraint parameter from RPY in degree
  moveit_msgs::OrientationConstraint orient_constr_definition(geometry_msgs::Vector3 RPY_orientation, std::string link_name,
   		  float toll_x = 0.1, float toll_y = 0.1, float toll_z = 0.1, float weight = 1.0, std::string header_frame_id = std_head_frame);

  //brief: Function to return the list of the available planners
  std::vector<std::string> getOmplPlannerList(void);

  //brief: Function to list all the possible basic solid shapes
  std::vector<std::string> get_possible_solid_shapes(void);

  //brief: Function to generate a collision object (SPHERE)
  moveit_msgs::CollisionObject collision_obj_generator(std::string id_collision_obj,
    		  geometry_msgs::Vector3 position, double radius, std::string solid_type = "SPHERE", std::string header_frame_id = std_head_frame);

  //brief: Function to generate a collision object (BOX) -- Using RPY in degree
  //       there is a function to know which is the planning frame "group.getPlanningFrame()" default = "/world"
  moveit_msgs::CollisionObject collision_obj_generator(std::string id_collision_obj, geometry_msgs::Vector3 position, geometry_msgs::Vector3 orientation,
		  geometry_msgs::Vector3 dimension, std::string solid_type = "BOX", std::string header_frame_id = std_head_frame);
  //brief: Function to generate a collision object (BOX) -- Using Quaternions
  moveit_msgs::CollisionObject collision_obj_generator(std::string id_collision_obj, geometry_msgs::Vector3 position, geometry_msgs::Quaternion quat,
		  geometry_msgs::Vector3 dimension, std::string solid_type = "BOX", std::string header_frame_id = std_head_frame);

  //brief: Function to generate a collision object (Cylinder or Cone) -- Using RPY in degree
  moveit_msgs::CollisionObject collision_obj_generator(std::string id_collision_obj, geometry_msgs::Vector3 position, geometry_msgs::Vector3 orientation,
		  double height, double radius, std::string solid_type = "CYLINDER", std::string header_frame_id = std_head_frame);
  //brief: Function to generate a collision object (Cylinder or Cone) -- Using Quaternions
  moveit_msgs::CollisionObject collision_obj_generator(std::string id_collision_obj, geometry_msgs::Vector3 position, geometry_msgs::Quaternion quat,
		  double height, double radius, std::string solid_type = "CYLINDER", std::string header_frame_id = std_head_frame);

  /* TO FINISH!
  //brief: Function to generate a collision object (BOX) -- Using RPY in degree
  //       define the position of the solid by the the centre of the solid base instead of the centre of the whole solid
  //       in "collision_obj_generator" the solid position is defined by the centre of it.
  moveit_msgs::CollisionObject collision_obj_generator_z(std::string id_collision_obj, geometry_msgs::Vector3 position, geometry_msgs::Vector3 orientation,
		  geometry_msgs::Vector3 dimension, std::string solid_type = "BOX", std::string header_frame_id = std_head_frame);
  */

  //brief: Functions to get the Endpoint State, Pose, Twist, Wrench
  //       at the beginning of your program you must have the following lines:
  //         ros::init(argc, argv, "ur_node_name");
  //         ros::NodeHandle node_handle("~");
  baxter_core_msgs::EndpointState getEndPointStateFromTopic(std::string right_left, ros::NodeHandle &nh);
  geometry_msgs::Pose getEePoseFromTopic(std::string right_left, ros::NodeHandle &nh);
  geometry_msgs::Twist getEeTwistFromTopic(std::string right_left, ros::NodeHandle &nh);
  geometry_msgs::Wrench getEeWrenchFromTopic(std::string right_left, ros::NodeHandle &nh);

  //brief: Functions to get the arm joints position, velocity, effort
  //       at the beginning of your program you must have the following lines:
  //         ros::init(argc, argv, "ur_node_name");
  //         ros::NodeHandle node_handle("~");
  sensor_msgs::JointState getBothArmJointValFromTopic(std::string right_left, ros::NodeHandle &nh);
  //One arm only!
  std::vector<double> getOneArmJointPositionFromTopic(std::string right_left, ros::NodeHandle &nh);
  std::vector<double> getOneArmJointVelocityFromTopic(std::string right_left, ros::NodeHandle &nh);
  std::vector<double> getOneArmJointEffortFromTopic(std::string right_left, ros::NodeHandle &nh);
  //Both arms (left joints before - right joints after in the vector)
  std::vector<double> getBothArmJointPositionFromTopic(ros::NodeHandle &nh);
  std::vector<double> getBothArmJointVelocityFromTopic(ros::NodeHandle &nh);
  std::vector<double> getBothArmJointEffortFromTopic(ros::NodeHandle &nh);



// End namespace "moveit_basics_functions"
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

  //brief: Function to define the Work Space Box
  //       This function has to be used when the planning frame is the world ("/world").
  bool setWorkSpaceBox(moveit::planning_interface::MoveGroup& obj, geometry_msgs::Vector3 min_XYZ, geometry_msgs::Vector3 max_XYZ);

  //brief: Function to change the planner ID among the ones available
  bool setPlanner(moveit::planning_interface::MoveGroup& obj, std::string new_planner);

  //brief: Function to change the planning time
  bool setPlanningTime(moveit::planning_interface::MoveGroup& obj, double new_time);

  //brief: Function to set the number of planning attempts - default = 1
  bool setPlanningAttempts(moveit::planning_interface::MoveGroup& obj, unsigned int attempts = baxter_attempts);

  //brief: Function to set the max velocity factor in (0.0 ; 1.0] - default = 1.0
  bool setMaxVelocityFactor(moveit::planning_interface::MoveGroup& obj, double val = baxter_max_factor);

  //brief: Function to set the max acceleration factor in (0.0 ; 1.0] - default = 1.0
  bool setMaxAccelerationFactor(moveit::planning_interface::MoveGroup& obj, double val = baxter_max_factor);

  //brief: Function to set the pose of the end-effector
  //       !Do not forget there is a function to generate a Pose msg (moveit_side_functions::makePose)
  bool setEePoseTarget(moveit::planning_interface::MoveGroup& obj,
		  geometry_msgs::Pose pose, std::string left_right = generic_str);

  //brief: Function to impose the joint value target directly
  //       Change "r_l_both" just in case both arms are controlled, but you want to move just one of them
  //       Do not forget to cluster the joint position with the function: 'moveit_side_functions::vector_two_cluster'.
  //       The values in the vector must go from the shoulder to the wrist.
  bool setJointValuesTarget(moveit::planning_interface::MoveGroup& obj,
		  std::vector<double> vector, std::string r_l_single = generic_str);

  //brief: Function to set the joint tolerance - data given in degree
  //       If you want to set the default tolerance do not put any tolerance value
  bool setJointTolerance(moveit::planning_interface::MoveGroup& obj, double toll = moveit_side_functions::rad2deg(baxter_def_joint_toll));

  //brief: Function to set the end-effector goal orientation tolerance - data given in degree
  //       If you want to set the default tolerance do not put any tolerance value
  bool setEeOrientationTolerance(moveit::planning_interface::MoveGroup& obj, double toll = moveit_side_functions::rad2deg(baxter_def_orient_toll));

  //brief: Function to set the end-effector goal position tolerance - data given in meter
  //       If you want to set the default tolerance do not put any tolerance value
  bool setEePositionTolerance(moveit::planning_interface::MoveGroup& obj, double toll = baxter_def_pos_toll);

  //brief: Function to set constraints
  bool setConstraint(moveit::planning_interface::MoveGroup& obj, moveit_msgs::Constraints constraint);

  //brief: Function to clear all the constraints
  bool clearConstraints(moveit::planning_interface::MoveGroup& obj);

  //brief: Function to plan and execute safely:
  //       if the path plan is not found it return a false msg and the robot do not move
  bool plan_execute_safely(moveit::planning_interface::MoveGroup& obj);

  //brief: Function to plan and execute safely, increasing the planning time gradually until "#define time_exit"
  //       if "restore_initial == true" the previous value is restored in the end
  bool plan_execute_safely_time_incresing(moveit::planning_interface::MoveGroup& obj, bool restore_initial = true);

  //brief: Function to plan and execute safely, increasing the planning attempts gradually until "#define att_exit"
  //       if "restore_initial == true" the default value (1) is restored in the end
  bool plan_execute_safely_attempt_incresing(moveit::planning_interface::MoveGroup& obj, bool restore_initial = true);

  //brief: Function to plan and move directly [not safe]
  void move_direct(moveit::planning_interface::MoveGroup& obj);

  //brief: Function to stop any trajectory motion, if one is running
  void motion_stop(moveit::planning_interface::MoveGroup& obj);


  ////// ENVIRONMENT FUNCTIONS //////
  // The object must be instantiated in the executable file using:
  // moveit::planning_interface::PlanningSceneInterface obj_name

  //brief: Function to add an object to the scene
  bool addObject(moveit::planning_interface::PlanningSceneInterface &interface, moveit_msgs::CollisionObject &coll_obj);

  //brief: Function to remove an object to the scene
  bool removeObject(moveit::planning_interface::PlanningSceneInterface &interface, moveit_msgs::CollisionObject &coll_obj);

  //brief: Function to attach a specific object to a moveit group
  //       obj_id = obj_in_the_scene.id
  bool attachObj2group(moveit::planning_interface::MoveGroup& group, std::string obj_id, std::string link_id = generic_str, double time2wait = std_time);

  //brief: Function to detach a specific object to a moveit group
  //       obj_id = obj_in_the_scene.id
  bool detachObj2group(moveit::planning_interface::MoveGroup& group, std::string obj_id, double time2wait = std_time);

  /* MOVEIT ERROR
  //brief: Function to get the current joint values of a specific arm or both, the default is all the joints values
  std::vector<double> getCurrentArmJointValues(moveit::planning_interface::MoveGroup& obj, std::string r_l_single = generic_str);

  //brief: Function to get the current Pose of a specific gripper, the default gripper is the right one
  geometry_msgs::Pose getCurrentGripperPose(moveit::planning_interface::MoveGroup& obj, std::string left_right = generic_str);

  //brief: Function to get the current orientation in quaternions of a specific gripper, the default gripper is the right one
  geometry_msgs::Quaternion getCurrentGripperQuaternion(moveit::planning_interface::MoveGroup& obj, std::string left_right);

  //brief: Function to get the current orientation in RPY of a specific gripper, the default gripper is the right one
  geometry_msgs::Vector3 getCurrentGripperRPY(moveit::planning_interface::MoveGroup& obj, std::string left_right);
  */


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
 * - getName **
 * - getNameTarget **
 * - getRobotName **
 * - getNodeHandle **
 * - getJointName //
 * - getLinkName //
 * - getActiveJoints **
 * - getJoints **
 * - getDefaultPlannerId **
 * - getCurrentJointValues //
 * - getCurrentState  **
 * - getCurrentPose //
 * - getCurrentRPY //
 *
 * - setPlannerId //
 * - setPlanningTime //
 * - setNumPlanningAttempts //
 * - setMaxVelocityScalingFactor //
 * - setMaxAccelerationScalingFactor //
 * - setWorkSpace //
 * - setStartState
 * - setStartStateToCurrentState
 * - setSupportSurfaceName **
 * - setJointValueTarget
 * - setApproximateJointValueTarget
 * - setPositionTarget
 * - setRPYTarget
 * - setOrientationTarget
 * - setPoseTarget //
 * - setPathConstraints //
 * - setGoalJointTolerance //
 * - setGoalPositionTolerance //
 * - setGoalOrientationTolerance //
 *
 * - clearPoseTarget **
 * - clearPathConstraints //
 *
 * - attachObject //
 * - detachObject //
 *
 * - computeCartesianPath
 * - Move //
 * - Plan //
 * - Execute //
 * - asyncMove **
 * - asyncExecute **
 * - Stop //
 * - Pick **
 * - Place **
 *
 */

