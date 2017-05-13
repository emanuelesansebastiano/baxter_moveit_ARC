/* Author: Emanuele Sansebastiano
   Desc: Class to encapsulate some side functions generated for the path planning part
         Amazon Robotics Challenge - Universidad Jaume I (Spain) [Competitor]
*/

// this pkg
#include <moveit_side_pkg/side_functions.h>

namespace moveit_side_functions
{
  double rad2deg(double rad)
  {
	  double pi = boost::math::constants::pi<double>();
	  int deg_pi = 180;
	  double deg = deg_pi*rad/pi;
	  return deg;
  }

  double deg2rad(double deg)
  {
	  double pi = boost::math::constants::pi<double>();
	  int deg_pi = 180;
	  double rad = pi*deg/deg_pi;
	  return rad;
  }

  double abs_f(double val)
  {
	  double abs_v;
	  if (val >= 0)
		  abs_v = val;
	  else
		  abs_v = -val;
	  return abs_v;
  }

  int round_f(double val)
  {
	  namespace msf = moveit_side_functions;
	  int temp = (int) val;
	  double temp1 = val - temp;
	  // if val is positive
	  if (msf::abs_f(temp1) >= 0.5 && temp1 >= 0)
		  temp = val +1;
	  // if val is negative
	  else if (msf::abs_f(temp1) >= 0.5 && temp1 < 0)
		  temp =  val -1;
	  //else {temp is already the integer part of val}
	  return temp;
  }

  std::vector<double> vector_two_cluster(std::vector<double> left, std::vector<double> right)
  {
	  int final_size = right.size() + left.size();
	  std::vector<double> final_vector; final_vector.resize(final_size);

	  for (int i = 0; i < left.size()-1; i++)
	  		  final_vector[i] = left[i];
	  for (int i = 0; i < right.size()-1; i++)
	  		  final_vector[i + left.size()] = right[i];

	  return final_vector;
  }

  geometry_msgs::Vector3 makeVector3(double x, double y, double z)
  {
	  geometry_msgs::Vector3 vector;
	  vector.x = x;
	  vector.y = y;
	  vector.z = z;

	  return vector;
  }

  geometry_msgs::Quaternion makeQuat(double w, double x, double y, double z)
  {
	  geometry_msgs::Quaternion quat;
	  quat.w = w;
	  quat.x = x;
	  quat.y = y;
	  quat.z = z;

  	  return quat;
  }

  geometry_msgs::Pose makePose(geometry_msgs::Quaternion orientation, geometry_msgs::Vector3 XYZ_location)
  {
    geometry_msgs::Pose pose;
    pose.orientation = orientation;
    pose.position.x = XYZ_location.x; pose.position.y = XYZ_location.y; pose.position.z = XYZ_location.z;

    return pose;
  }
  //RPY_orientation in degree
  geometry_msgs::Pose makePose(geometry_msgs::Vector3 RPY_orientation, geometry_msgs::Vector3 XYZ_location)
  {
    geometry_msgs::Pose pose;
  	pose.orientation = moveit_side_functions::RPY2Quat(RPY_orientation, false);
  	pose.position.x = XYZ_location.x; pose.position.y = XYZ_location.y; pose.position.z = XYZ_location.z;

  	return pose;
  }

  geometry_msgs::PoseStamped Pose2PoseStamped( geometry_msgs::Pose old_pose)
  {
	  geometry_msgs::PoseStamped new_pose_s;
	  new_pose_s.pose.position.x = old_pose.position.x;
	  new_pose_s.pose.position.y = old_pose.position.y;
	  new_pose_s.pose.position.z = old_pose.position.z;
	  new_pose_s.pose.orientation.w = old_pose.orientation.w;
	  new_pose_s.pose.orientation.x = old_pose.orientation.x;
	  new_pose_s.pose.orientation.y = old_pose.orientation.y;
	  new_pose_s.pose.orientation.z = old_pose.orientation.z;

	  return new_pose_s;
  }

  geometry_msgs::Pose PoseStamped2Pose( geometry_msgs::PoseStamped old_pose_s)
  {
	  geometry_msgs::Pose new_pose;
	  new_pose.position.x = old_pose_s.pose.position.x;
	  new_pose.position.y = old_pose_s.pose.position.y;
	  new_pose.position.z = old_pose_s.pose.position.z;
	  new_pose.orientation.w = old_pose_s.pose.orientation.w;
	  new_pose.orientation.x = old_pose_s.pose.orientation.x;
	  new_pose.orientation.y = old_pose_s.pose.orientation.y;
	  new_pose.orientation.z = old_pose_s.pose.orientation.z;

	  return new_pose;
  }

  /* Angles names:
   * Tilt - Bank - Roll - Psi - around x
   * Elevation - Attitude - Pitch - Phi - around Y
   * Azimuth - Heading - Yaw - Theta - around Z
  */
  geometry_msgs::Quaternion RPY2Quat(geometry_msgs::Vector3 RPY, bool RPY_rad, bool turn_corr)
  {
	  namespace msf = moveit_side_functions;
	  geometry_msgs::Quaternion Quat;
	  const int arr_len = 3;

	  double v[arr_len] = {0,0,0};
	  v[0] = RPY.x; v[1] = RPY.y; v[2] = RPY.z;

	  // convert degree to radiant
	  if (!RPY_rad){
		  for (int i = 0; i < arr_len; i++)
		  {
			  v[i] = msf::deg2rad(v[i]);
		  }
	  }

	  // keep the angles values in (-pi ; pi]
	  if (turn_corr)
	  {
		  double pi = boost::math::constants::pi<double>();
		  for (int i = 0; i < arr_len; i++)
		  {
			  if (v[i] == -pi){
				  v[i] = pi;
			  }
			  // absolute converter
			  if (msf::abs_f(v[i]) > pi){
				  v[i] = v[i] - msf::round_f(v[i]/(2*pi))*(2*pi);
				  //std::cout << msf::round_f(v[i]/(2*pi)) << std::endl;
			  }
		  }
	  }

	  // pre-definition
	  double c1 = cos(v[2]/2);
	  double c2 = cos(v[1]/2);
	  double c3 = cos(v[0]/2);
	  double s1 = sin(v[2]/2);
	  double s2 = sin(v[1]/2);
	  double s3 = sin(v[0]/2);

	  Quat.w = c1*c2*c3 - s1*s2*s3;
	  Quat.x = s1*s2*c3 + c1*c2*s3;
	  Quat.y = c1*s2*c3 - s1*c2*s3;
	  Quat.z = s1*c2*c3 + c1*s2*s3;

	  return Quat;
  }

  double Quat_diff(geometry_msgs::Quaternion a, geometry_msgs::Quaternion b)
  {
	  namespace msf = moveit_side_functions;
	  const int val_quat = 4;
	  double temp[val_quat];
	  double sum = 0.0;

	  temp[0] = msf::abs_f(a.x - b.x);
	  temp[1] = msf::abs_f(a.y - b.y);
	  temp[2] = msf::abs_f(a.z - b.z);
	  temp[3] = msf::abs_f(a.w - b.w);

	  for (int i; i < val_quat; i++)
	  {
		  sum += temp[i];
	  }

	  return sum;
  }


// End namespace "moveit_side_functions"
}


namespace moveit_basics_functions
{

  geometry_msgs::Vector3 vertical_orientation_x(void)
  {
	  geometry_msgs::Vector3 RPY_up;
   	  RPY_up.x = 180.0; RPY_up.y = 0.0; RPY_up.z = 0.0;
      return RPY_up;
  }

  geometry_msgs::Vector3 vertical_orientation_y(void)
  {
	  geometry_msgs::Vector3 RPY_up;
   	  RPY_up.x = 0.0; RPY_up.y = 180.0; RPY_up.z = 0.0;
      return RPY_up;
  }

  geometry_msgs::Pose up_position(geometry_msgs::Pose ee_curr_pos, double curr_z, double distance)
  {
	  namespace msf = moveit_side_functions;

	  // ee_target position definition
	  geometry_msgs::Pose ee_target;
	  ee_target = ee_curr_pos;
	  ee_target.position.z += distance;

      // ee_target orientation definition
	  // definition of the two possible final orientation
	  geometry_msgs::Vector3 RPY_up_x = moveit_basics_functions::vertical_orientation_x();
	  geometry_msgs::Vector3 RPY_up_y = moveit_basics_functions::vertical_orientation_y();
	  RPY_up_x.z = curr_z;
	  RPY_up_y.z = curr_z;
	  geometry_msgs::Quaternion Quat_up_x = msf::RPY2Quat(RPY_up_x);
	  geometry_msgs::Quaternion Quat_up_y = msf::RPY2Quat(RPY_up_y);
      // definition or the current orientation
	  geometry_msgs::Quaternion ee_curr_or = ee_curr_pos.orientation;

	  if (msf::Quat_diff(ee_curr_or, Quat_up_x) >= msf::Quat_diff(ee_curr_or, Quat_up_y)){
		  ee_target.orientation = Quat_up_y;
	  }else{
		  ee_target.orientation = Quat_up_x;
	  }

	  return ee_target;
  }

  moveit_msgs::OrientationConstraint orient_constr_definition(geometry_msgs::Quaternion orientation, std::string link_name, float toll_x, float toll_y, float toll_z, float weight, std::string header_frame_id)
  {
	  moveit_msgs::OrientationConstraint orient_constr;
	  orient_constr.orientation = orientation;
	  orient_constr.link_name = link_name;
	  orient_constr.absolute_x_axis_tolerance = toll_x;
	  orient_constr.absolute_y_axis_tolerance = toll_y;
	  orient_constr.absolute_z_axis_tolerance = toll_z;
	  orient_constr.weight = weight;
	  orient_constr.header.frame_id = header_frame_id;

	  return orient_constr;
  }
  //RPY_orientation in degree
  moveit_msgs::OrientationConstraint orient_constr_definition(geometry_msgs::Vector3 RPY_orientation, std::string link_name, float toll_x, float toll_y, float toll_z, float weight, std::string header_frame_id)
  {
      moveit_msgs::OrientationConstraint orient_constr;
  	  orient_constr.orientation = moveit_side_functions::RPY2Quat(RPY_orientation, false);
  	  orient_constr.link_name = link_name;
  	  orient_constr.absolute_x_axis_tolerance = toll_x;
  	  orient_constr.absolute_y_axis_tolerance = toll_y;
  	  orient_constr.absolute_z_axis_tolerance = toll_z;
  	  orient_constr.weight = weight;
  	  orient_constr.header.frame_id = header_frame_id;

  	  return orient_constr;
  }

  std::vector<std::string> getOmplPlannerList(void)
  {
	  std::vector<std::string> planner_list;
	  planner_list.resize(8);
	  std::string base = "ConfigDefault";
	  planner_list[0] = "BKPIECE" + base;
	  planner_list[1] = "EST" + base;
	  planner_list[2] = "KPIECE" + base;
	  planner_list[3] = "LBKPIECE" + base;
	  planner_list[4] = "RRTConnect" + base;
	  planner_list[5] = "RRTStar" + base;
	  planner_list[6] = "RRT" + base;
	  planner_list[7] = "SBL" + base;

	  return planner_list;
  }

  std::vector<std::string> get_possible_solid_shapes(void)
  {
	  std::vector<std::string> solids;
	  solids[0] = "BOX";
	  solids[1] = "SPHERE";
	  solids[2] = "CYLINDER";
	  solids[3] = "CONE";

	  return solids;
  }

  //Collision object generator: SPHERE
  moveit_msgs::CollisionObject collision_obj_generator(std::string id_collision_obj, geometry_msgs::Vector3 position, double radius, std::string header_frame_id , std::string solid_type)
{
	  //Collision object definition
	  moveit_msgs::CollisionObject collision_object;
	  collision_object.id = id_collision_obj;
	  //remember to use the function "move_group.getPlanningFrame()" to get the header frame.
	  collision_object.header.frame_id = header_frame_id;

	  //solid definition
	  shape_msgs::SolidPrimitive output_solid;
	  output_solid.type = shape_msgs::SolidPrimitive::SPHERE;
	  output_solid.dimensions.resize(1);
	  output_solid.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS] = radius;

	  //solid pose definition
	  geometry_msgs::Pose solid_pose;
	  //the orientation is not important
	  solid_pose.orientation.w = 1.0;
	  solid_pose.position.x = position.x; solid_pose.position.y = position.y; solid_pose.position.z = position.z;

	  collision_object.primitives.push_back(output_solid);
	  collision_object.primitives.push_back(output_solid);

	  return collision_object;
  }

  //Collision object generator: BOX -- Using RPY in degree
  moveit_msgs::CollisionObject collision_obj_generator(std::string id_collision_obj, geometry_msgs::Vector3 orientation, geometry_msgs::Vector3 position, geometry_msgs::Vector3 dimension, std::string header_frame_id , std::string solid_type)
  {
 	  //Collision object definition
  	  moveit_msgs::CollisionObject collision_object;
  	  collision_object.id = id_collision_obj;
  	  //remember to use the function "move_group.getPlanningFrame()" to get the header frame.
  	  collision_object.header.frame_id = header_frame_id;

  	  //solid definition
  	  shape_msgs::SolidPrimitive output_solid;
  	  output_solid.type = shape_msgs::SolidPrimitive::BOX;
  	  output_solid.dimensions.resize(3);
  	  output_solid.dimensions[shape_msgs::SolidPrimitive::BOX_X] = dimension.x;
  	  output_solid.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = dimension.y;
  	  output_solid.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = dimension.z;

  	  //solid pose definition
  	  geometry_msgs::Pose solid_pose;
  	  //the orientation is not important
  	  solid_pose.orientation = moveit_side_functions::RPY2Quat(orientation, false);
  	  solid_pose.position.x = position.x; solid_pose.position.y = position.y; solid_pose.position.z = position.z;

  	  collision_object.primitives.push_back(output_solid);
  	  collision_object.primitives.push_back(output_solid);

  	  return collision_object;
    }
  //Collision object generator: BOX -- Using Quaternions
  moveit_msgs::CollisionObject collision_obj_generator(std::string id_collision_obj, geometry_msgs::Quaternion quat, geometry_msgs::Vector3 position, geometry_msgs::Vector3 dimension, std::string header_frame_id , std::string solid_type)
  {
 	  //Collision object definition
  	  moveit_msgs::CollisionObject collision_object;
  	  collision_object.id = id_collision_obj;
  	  //remember to use the function "move_group.getPlanningFrame()" to get the header frame.
  	  collision_object.header.frame_id = header_frame_id;

  	  //solid definition
  	  shape_msgs::SolidPrimitive output_solid;
  	  output_solid.type = shape_msgs::SolidPrimitive::BOX;
  	  output_solid.dimensions.resize(3);
  	  output_solid.dimensions[shape_msgs::SolidPrimitive::BOX_X] = dimension.x;
  	  output_solid.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = dimension.y;
  	  output_solid.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = dimension.z;

  	  //solid pose definition
  	  geometry_msgs::Pose solid_pose;
  	  //the orientation is not important
  	  solid_pose.orientation = quat;
  	  solid_pose.position.x = position.x; solid_pose.position.y = position.y; solid_pose.position.z = position.z;

  	  collision_object.primitives.push_back(output_solid);
  	  collision_object.primitives.push_back(output_solid);

  	  return collision_object;
    }

  //Collision object generator: CYLINDER or CONE -- Using RPY in degree
  moveit_msgs::CollisionObject collision_obj_generator(std::string id_collision_obj, geometry_msgs::Vector3 orientation, geometry_msgs::Vector3 position, double height, double radius, std::string header_frame_id , std::string solid_type)
  {
 	  //Collision object definition
  	  moveit_msgs::CollisionObject collision_object;
  	  collision_object.id = id_collision_obj;
  	  //remember to use the function "move_group.getPlanningFrame()" to get the header frame.
  	  collision_object.header.frame_id = header_frame_id;

  	  //solid definition
  	  shape_msgs::SolidPrimitive output_solid;

  	  output_solid.type = shape_msgs::SolidPrimitive::CYLINDER;
  	  if(solid_type == "CONE")
  		  output_solid.type = shape_msgs::SolidPrimitive::CONE;
  	  else
  		  std::cout << "Warning: A default cylinder has been created, but it was defined wrongly. '" << solid_type << "' is valid!" << std::endl << "Just 'CYLINDER' and 'CONE' are valid!" << std::endl;

  	  output_solid.dimensions.resize(2);
  	  output_solid.dimensions[0] = height;
  	  output_solid.dimensions[1] = radius;

  	  //solid pose definition
  	  geometry_msgs::Pose solid_pose;
  	  //the orientation is not important
  	  solid_pose.orientation = moveit_side_functions::RPY2Quat(orientation, false);
  	  solid_pose.position.x = position.x; solid_pose.position.y = position.y; solid_pose.position.z = position.z;

  	  collision_object.primitives.push_back(output_solid);
  	  collision_object.primitives.push_back(output_solid);

  	  return collision_object;
    }
  //Collision object generator: CYLINDER or CONE -- Using Quaternions
  moveit_msgs::CollisionObject collision_obj_generator(std::string id_collision_obj, geometry_msgs::Quaternion quat, geometry_msgs::Vector3 position, double height, double radius, std::string header_frame_id , std::string solid_type)
  {
 	  //Collision object definition
  	  moveit_msgs::CollisionObject collision_object;
  	  collision_object.id = id_collision_obj;
  	  //remember to use the function "move_group.getPlanningFrame()" to get the header frame.
  	  collision_object.header.frame_id = header_frame_id;

  	  //solid definition
  	  shape_msgs::SolidPrimitive output_solid;

  	  output_solid.type = shape_msgs::SolidPrimitive::CYLINDER;
  	  if(solid_type == "CONE")
  		  output_solid.type = shape_msgs::SolidPrimitive::CONE;
  	  else
  		  std::cout << "Warning: A default cylinder has been created, but it was defined wrongly. '" << solid_type << "' is valid!" << std::endl << "Just 'CYLINDER' and 'CONE' are valid!" << std::endl;

  	  output_solid.dimensions.resize(2);
  	  output_solid.dimensions[0] = height;
  	  output_solid.dimensions[1] = radius;

  	  //solid pose definition
  	  geometry_msgs::Pose solid_pose;
  	  //the orientation is not important
  	  solid_pose.orientation = quat;
  	  solid_pose.position.x = position.x; solid_pose.position.y = position.y; solid_pose.position.z = position.z;

  	  collision_object.primitives.push_back(output_solid);
  	  collision_object.primitives.push_back(output_solid);

  	  return collision_object;
    }


// End namespace "moveit_basics_functions"
}


namespace obj_functions
{
  ////// ROBOT FUNCTIONS //////
  std::vector<std::string> GroupNameAvailable(void)
  {
	  std::vector<std::string> names;
	  names[0] = "left_arm";
	  names[1] = "right_arm";
	  // the option "both_arms" must be always the last one
	  names[2] = "both_arms";

	  return names;
  }

  std::vector<std::string> getLinkNames(moveit::planning_interface::MoveGroup& obj)
  {
	  std::vector<std::string> LinkNames = obj.getLinkNames();
	  return LinkNames;
  }

  std::vector<std::string> getJointNames(moveit::planning_interface::MoveGroup& obj)
  {
  	  std::vector<std::string> JointNames = obj.getJointNames();
  	  return JointNames;
  }

  std::vector<double> getCurrentArmJointValues(moveit::planning_interface::MoveGroup& obj, std::string r_l_single)
  {
	  std::vector<std::string> pos_groups = obj_functions::GroupNameAvailable();
	  std::vector<std::string> joint_names = getJointNames(obj);
	  std::vector<double> joint_val;

	  //if two arms and I just want to know one of them
	  if(obj.getName() == pos_groups[pos_groups.size() -1] && (r_l_single == "right" || r_l_single == "left")){
	  	  std::cout << "Even if the group " << obj.getName() << " is using both arms, just the joint values of the " << r_l_single << " arm are returned." << std::endl;
	  	  std::vector<double> joint_val_temp = obj.getCurrentJointValues();
	  	  int size1arm = joint_val_temp.size()/2;
	  	  int count = 0;
	  	  for(int i = 0; i < size1arm; i++)
	  	  {
	  		if(joint_names[i].find(r_l_single) >= 0){
	  			joint_val[count] = joint_val_temp[i];
	  			count++;
	  		}
	  	  }
	  }else{
		  joint_val = obj.getCurrentJointValues();
	  }

	  return joint_val;
  }

  geometry_msgs::Pose getCurrentGripperPose(moveit::planning_interface::MoveGroup& obj, std::string left_right)
  {
	  std::vector<std::string> pos_groups = obj_functions::GroupNameAvailable();
  	  std::string gripper;
  	  if(obj.getName() == pos_groups[pos_groups.size() -1] && (left_right != "right" || left_right != "left")){
  		  std::cout << "Warning: insert correctly to which gripper you want to get the current pose. ";
  		  std::cout << "Insert 'right' or 'left', otherwise this function will not return the right one by default." << std::endl;
  		  left_right = "right";
  		  gripper = left_right + "_gripper";
  	  }else if(obj.getName() == pos_groups[pos_groups.size() -1] && (left_right == "right" || left_right == "left")){
  		  gripper = left_right + "_gripper";
  	  }else if(obj.getName() != pos_groups[pos_groups.size() -1]){
  		  gripper = obj.getEndEffectorLink();
  	  }

  	  geometry_msgs::PoseStamped poseStamp2return = obj.getCurrentPose(gripper);
  	  geometry_msgs::Pose pose2return = moveit_side_functions::PoseStamped2Pose(poseStamp2return);
	  return pose2return;
  }

  geometry_msgs::Quaternion getCurrentGripperQuaternion(moveit::planning_interface::MoveGroup& obj, std::string left_right)
  {
	  geometry_msgs::Pose temp_pose = obj_functions::getCurrentGripperPose(obj, left_right);
	  geometry_msgs::Quaternion quat2return = temp_pose.orientation;
	  return quat2return;
  }

  geometry_msgs::Vector3 getCurrentGripperRPY(moveit::planning_interface::MoveGroup& obj, std::string left_right)
    {
  	  std::vector<std::string> pos_groups = obj_functions::GroupNameAvailable();
    	  std::string gripper;
    	  if(obj.getName() == pos_groups[pos_groups.size() -1] && (left_right != "right" || left_right != "left")){
    		  std::cout << "Warning: insert correctly to which gripper you want to get the current pose. ";
    		  std::cout << "Insert 'right' or 'left', otherwise this function will not return the right one by default." << std::endl;
    		  left_right = "right";
    		  gripper = left_right + "_gripper";
    	  }else if(obj.getName() == pos_groups[pos_groups.size() -1] && (left_right == "right" || left_right == "left")){
    		  gripper = left_right + "_gripper";
    	  }else if(obj.getName() != pos_groups[pos_groups.size() -1]){
    		  gripper = obj.getEndEffectorLink();
    	  }

    	  geometry_msgs::Vector3 RPY2return;
    	  std::vector<double> temp_RPY = obj.getCurrentRPY(gripper);
    	  RPY2return.x = temp_RPY[0];
    	  RPY2return.y = temp_RPY[1];
    	  RPY2return.z = temp_RPY[2];
  	  return RPY2return;
    }

  //NO effective check
  bool setWorkSpaceBox(moveit::planning_interface::MoveGroup& obj, geometry_msgs::Vector3 min_XYZ, geometry_msgs::Vector3 max_XYZ)
  {
	  if(obj.getPlanningFrame() != "/world")
		  std::cout << "Warning: you are not planning on the '/world' frame. You are planning on the '" << obj.getPlanningFrame() << "'. Be careful!" << std::endl;

	  obj.setWorkspace(min_XYZ.x, min_XYZ.y, min_XYZ.z, max_XYZ.x, max_XYZ.y, max_XYZ.z);

	  return true;
  }

  //NO effective check (Planner existance check)
  bool setPlanner(moveit::planning_interface::MoveGroup& obj, std::string new_planner)
  {
	  bool success = false;
	  std::string obj_group = obj.getName();
	  std::string planner_ID = obj.getDefaultPlannerId(obj_group);
	  std::vector<std::string> planner_list = moveit_basics_functions::getOmplPlannerList();
	  int temp = planner_list.size();
	  for (int i = 0; i < temp; i++){
		  if (planner_list[i] == new_planner){
			  success = true;
		      break;
		  }
	  }
	  if (success){
		  obj.setPlannerId(new_planner);
		  std::cout << "Done correctly: The new planner for the group " << obj_group << " is " << new_planner << "." << std::endl;
      }else{
		  std::cout << "Warning: The planner chosen do not exit in OMPL Default. Call the function 'moveit_basics_functions::getOmplPlannerList() to check the planners available." << std::endl;
	      std::cout << "Warning: The planner for the group " << obj_group << " is still the last one successfully set." << std::endl;
     }

	 return success;
  }

  //YES effective check
  bool setPlanningTime(moveit::planning_interface::MoveGroup& obj, double time)
  {
	  bool success = false;
	  if (time < 0.0){
		  std::cout << "Warning: The planning time cannot be negative! Previous value is maintained." << std::endl;
	  }else{
		  int exit = 0;
		  while (!success || exit >= 5){
			  obj.setPlanningTime(time);
			  ros::Duration(0.02).sleep();
			  if(obj.getPlanningTime() == time){
				  success = true;
				  std::cout << "Done correctly: The new planning time of the group " << obj.getName() << " is " << time << "." << std::endl;
			  }else{
				  exit++;
			  }
		  }
	  }

	  return success;
  }

  //NO effective check
  bool setPlanningAttempts(moveit::planning_interface::MoveGroup& obj, unsigned int attempts)
  {
	  obj.setNumPlanningAttempts(attempts);

	  return true;
  }

  //NO effective check
  bool setMaxVelocityFactor(moveit::planning_interface::MoveGroup& obj, double val)
  {
	  if(val > 0.0 && val <= 1.0){
		  obj.setMaxVelocityScalingFactor(val);
		  return true;
	  }else{
		  std::cout << "Warning: the factor must be in the interval (0.0; 1.0]! You inserted " << val << "." <<std::endl;
		  return false;
	  }
  }

  //NO effective check
  bool setMaxAccelerationFactor(moveit::planning_interface::MoveGroup& obj, double val)
  {
	  if(val > 0.0 && val <= 1.0){
		  obj.setMaxAccelerationScalingFactor(val);
		  return true;
	  }else{
		  std::cout << "Warning: the factor must be in the interval (0.0; 1.0]! You inserted " << val << "." <<std::endl;
		  return false;
	  }
  }

  //--- effective check
  bool setEePoseTarget(moveit::planning_interface::MoveGroup& obj, geometry_msgs::Pose pose, std::string left_right)
  {
	  bool success = true;
	  std::vector<std::string> pos_groups = obj_functions::GroupNameAvailable();
	  std::string gripper;
	  if(obj.getName() == pos_groups[pos_groups.size() -1] && (left_right != "right" || left_right != "left")){
		  std::cout << "Warning: insert correctly to which gripper you want to assign to that pose msg. ";
		  std::cout << "Insert 'right' or 'left', otherwise this function will not return positive value." << std::endl;
		  return false;
	  }else if(obj.getName() == pos_groups[pos_groups.size() -1] && (left_right == "right" || left_right == "left")){
		  gripper = left_right + "_gripper";
	  }else if(obj.getName() != pos_groups[pos_groups.size() -1]){
		  gripper = obj.getEndEffectorLink();
	  }
	  obj.setPoseTarget(pose, gripper);

	  return success;
  }

  //--- effective check
  bool setJointValuesTarget(moveit::planning_interface::MoveGroup& obj, std::vector<double> vector, std::string r_l_single)
  {
	  // All the possible groups
	  std::vector<std::string> pos_groups = obj_functions::GroupNameAvailable();
	  // Current object group name
	  std::string GroupName = obj.getName();
	  // Current group joints names
	  std::vector<std::string> jointsName = obj.getJointNames();
	  // Number possible groups
	  int num_groups = pos_groups.size();
	  // Number of joints
	  int size = jointsName.size();

	  // CHECK AND CONTROL
	  if(GroupName == pos_groups[num_groups-1]){
		  if(r_l_single != "right" || r_l_single != "left")
			  return false;
		  else if(r_l_single == "right" || r_l_single == "left")
			  size = size/2;
	  }
	  if(size != vector.size())
		  return false;

	  // if I want to change the position of just one arm and I am controlling both arms
	  if(GroupName == pos_groups[num_groups-1] && size != jointsName.size()){

		  int count = 0;
		  for(int i = 0; i < jointsName.size(); i++){
			  if(jointsName[i].find(r_l_single) >= 0){
				  obj.setJointValueTarget(jointsName[i], vector[count]);
				  count++;
			  }
		  }
	  // if I want to change the position of all the arms I am controlling
	  }else{
		  obj.setJointValueTarget(vector);
	  }

	  return true;
  }

  //NO effective check
  bool clearConstraints(moveit::planning_interface::MoveGroup& obj)
  {
	  obj.clearPathConstraints();

	  return true;
  }


  void move_f(moveit::planning_interface::MoveGroup& obj)
  {
	  obj.move();
  }



  ////// ENVIRONMENT FUNCTIONS //////
  //NO effective check
  bool addObject(moveit::planning_interface::PlanningSceneInterface &interface, moveit_msgs::CollisionObject &coll_obj)
  {
	  coll_obj.operation = coll_obj.ADD;
	  interface.applyCollisionObject(coll_obj);

	  return true;
  }

  //NO effective check
  bool removeObject(moveit::planning_interface::PlanningSceneInterface &interface, moveit_msgs::CollisionObject &coll_obj)
  {
	  coll_obj.operation = coll_obj.REMOVE;
	  interface.applyCollisionObject(coll_obj);

	  return true;
  }

  //NO effective check
  bool attachObj2group(moveit::planning_interface::MoveGroup& group, std::string obj_id, double time2wait)
  {
	  group.attachObject(obj_id);
	  ros::Duration(time2wait).sleep();

	  return true;
  }

  //NO effective check
  bool detachObj2group(moveit::planning_interface::MoveGroup& group, std::string obj_id, double time2wait)
  {
  	  group.detachObject(obj_id);
  	  ros::Duration(time2wait).sleep();

  	  return true;
  }


// End namespace "obj_functions"
}

