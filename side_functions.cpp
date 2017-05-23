/* Author: Emanuele Sansebastiano
   Desc: Class to encapsulate some side functions generated for the path planning part
         Amazon Robotics Challenge - Universidad Jaume I (Spain) [Competitor]
*/

// this pkg
#include <moveit_side_pkg/side_functions.h>


namespace moveit_side_functions
{
  int _get_attempt_val(int val)
  {
	  return val;
  }

  void standardSleep(double sleep_time)
  {
	  ros::Duration(sleep_time).sleep();
  }

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
	  int temp = (int) val;
	  double temp1 = val - temp;
	  // if val is positive
	  if (abs_f(temp1) >= 0.5 && temp1 >= 0)
		  temp = val +1;
	  // if val is negative
	  else if (abs_f(temp1) >= 0.5 && temp1 < 0)
		  temp =  val -1;
	  //else {temp is already the integer part of val}
	  return temp;
  }

  int decimal_shift(double val2shift, int decimal_considered)
  {
	  int val2return = val2shift*std::pow(10,decimal_considered);
	  return val2return;
  }

  std::vector<double> vector_two_cluster(std::vector<double> left, std::vector<double> right)
  {
	  int final_size = right.size() + left.size();
	  std::vector<double> final_vector; final_vector.resize(final_size);

	  for (int i = 0; i < left.size(); i++)
	  		  final_vector[i] = left[i];
	  for (int i = 0; i < right.size(); i++)
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

  /* Angles names:
   * Tilt - Bank - Roll - Psi - around x
   * Elevation - Attitude - Pitch - Phi - around Y
   * Azimuth - Heading - Yaw - Theta - around Z
  */
  geometry_msgs::Quaternion RPY2Quat(geometry_msgs::Vector3 RPY, bool RPY_rad, bool turn_corr)
  {
	  geometry_msgs::Quaternion Quat;
	  const int arr_len = 3;

	  double v[arr_len] = {0,0,0};
	  v[0] = RPY.x; v[1] = RPY.y; v[2] = RPY.z;

	  // convert degree to radiant
	  if (!RPY_rad){
		  for (int i = 0; i < arr_len; i++)
		  {
			  v[i] = deg2rad(v[i]);
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
			  if (abs_f(v[i]) > pi){
				  v[i] = v[i] - round_f(v[i]/(2*pi))*(2*pi);
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
  	pose.orientation = RPY2Quat(RPY_orientation, false);
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

  double Quat_diff(geometry_msgs::Quaternion a, geometry_msgs::Quaternion b)
  {
	  const int val_quat = 4;
	  double temp[val_quat];
	  double sum = 0.0;

	  temp[0] = abs_f(a.x - b.x);
	  temp[1] = abs_f(a.y - b.y);
	  temp[2] = abs_f(a.z - b.z);
	  temp[3] = abs_f(a.w - b.w);

	  for (int i; i < val_quat; i++)
	  {
		  sum += temp[i];
	  }

	  return sum;
  }

  bool PoseEquivalence(geometry_msgs::Pose A, geometry_msgs::Pose B, int decimal_considered)
  {
	  bool equal = false;
	  if(decimal_considered < 0)
	  {
		  if(A.position.x == B.position.x &&
			 A.position.y == B.position.y &&
			 A.position.z == B.position.z &&
			 A.orientation.x == B.orientation.x &&
			 A.orientation.y == B.orientation.y &&
			 A.orientation.z == B.orientation.z &&
			 A.orientation.w == B.orientation.w)

			  equal = true;
	  }else{
		  int dc = decimal_considered;
		  if(decimal_shift(A.position.x, dc) == decimal_shift(B.position.x, dc) &&
			 decimal_shift(A.position.y, dc) == decimal_shift(B.position.y, dc) &&
			 decimal_shift(A.position.z, dc) == decimal_shift(B.position.z, dc) &&
			 decimal_shift(A.orientation.x, dc) == decimal_shift(B.orientation.x, dc) &&
			 decimal_shift(A.orientation.y, dc) == decimal_shift(B.orientation.y, dc) &&
			 decimal_shift(A.orientation.z, dc) == decimal_shift(B.orientation.z, dc) &&
			 decimal_shift(A.orientation.w, dc) == decimal_shift(B.orientation.w, dc))

			  equal = true;
	  }

	  return equal;
  }

  bool CheckTopicExistence(std::string topic2check)
  {
	  bool success = false;
	  //get all topics in the system running
	  ros::master::V_TopicInfo master_topics;
	  ros::master::getTopics(master_topics);
	  for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++) {
	      ros::master::TopicInfo& info = *it;
	      if(info.name == topic2check)
	      {
	    	  success = true;
	    	  break;
	      }
	  }

	  return success;
  }

// End namespace "moveit_side_functions"
}


namespace moveit_basics_functions
{
  //Global variable for this namespace
  baxter_core_msgs::EndpointState gl_end_point_state;
  sensor_msgs::JointState gl_joints_state;

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
	  geometry_msgs::Vector3 RPY_up_x = vertical_orientation_x();
	  geometry_msgs::Vector3 RPY_up_y = vertical_orientation_y();
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
	  solids.resize(4);
	  solids[0] = "BOX";
	  solids[1] = "SPHERE";
	  solids[2] = "CYLINDER";
	  solids[3] = "CONE";

	  return solids;
  }

  //Collision object generator: SPHERE
  moveit_msgs::CollisionObject collision_obj_generator(std::string id_collision_obj, geometry_msgs::Vector3 position, double radius, std::string solid_type, std::string header_frame_id)
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
	  collision_object.primitive_poses.push_back(solid_pose);

	  return collision_object;
  }

  //Collision object generator: BOX -- Using RPY in degree
  moveit_msgs::CollisionObject collision_obj_generator(std::string id_collision_obj, geometry_msgs::Vector3 position, geometry_msgs::Vector3 orientation, geometry_msgs::Vector3 dimension, std::string solid_type, std::string header_frame_id)
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
  	  collision_object.primitive_poses.push_back(solid_pose);

  	  return collision_object;
   }
  //Collision object generator: BOX -- Using Quaternions
  moveit_msgs::CollisionObject collision_obj_generator(std::string id_collision_obj, geometry_msgs::Vector3 position, geometry_msgs::Quaternion quat, geometry_msgs::Vector3 dimension, std::string solid_type, std::string header_frame_id)
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
  	  collision_object.primitive_poses.push_back(solid_pose);

  	  return collision_object;
   }

  //Collision object generator: CYLINDER or CONE -- Using RPY in degree
  moveit_msgs::CollisionObject collision_obj_generator(std::string id_collision_obj, geometry_msgs::Vector3 position, geometry_msgs::Vector3 orientation, double height, double radius, std::string solid_type, std::string header_frame_id)
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
  	  else if(solid_type != "CONE" && solid_type != "CYLINDER")
  		  std::cout << "Warning: A default cylinder has been created, but the solid type inserted was not valid. '" << solid_type << "' is not valid!" << std::endl << "Just 'CYLINDER' and 'CONE' are valid!" << std::endl;

  	  output_solid.dimensions.resize(2);
  	  output_solid.dimensions[0] = height;
  	  output_solid.dimensions[1] = radius;

  	  //solid pose definition
  	  geometry_msgs::Pose solid_pose;
  	  //the orientation is not important
  	  solid_pose.orientation = moveit_side_functions::RPY2Quat(orientation, false);
  	  solid_pose.position.x = position.x; solid_pose.position.y = position.y; solid_pose.position.z = position.z;

  	  collision_object.primitives.push_back(output_solid);
  	  collision_object.primitive_poses.push_back(solid_pose);

  	  return collision_object;
   }
  //Collision object generator: CYLINDER or CONE -- Using Quaternions
  moveit_msgs::CollisionObject collision_obj_generator(std::string id_collision_obj, geometry_msgs::Vector3 position, geometry_msgs::Quaternion quat, double height, double radius, std::string solid_type, std::string header_frame_id)
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
  	  else if(solid_type != "CONE" && solid_type != "CYLINDER")
  		  std::cout << "Warning: A default cylinder has been created, but the solid type inserted was not valid. '" << solid_type << "' is not valid!" << std::endl << "Just 'CYLINDER' and 'CONE' are valid!" << std::endl;

  	  output_solid.dimensions.resize(2);
  	  output_solid.dimensions[0] = height;
  	  output_solid.dimensions[1] = radius;

  	  //solid pose definition
  	  geometry_msgs::Pose solid_pose;
  	  //the orientation is not important
  	  solid_pose.orientation = quat;
  	  solid_pose.position.x = position.x; solid_pose.position.y = position.y; solid_pose.position.z = position.z;

  	  collision_object.primitives.push_back(output_solid);
  	  collision_object.primitive_poses.push_back(solid_pose);

  	  return collision_object;
   }

  /* TO FINISH!
  moveit_msgs::CollisionObject collision_obj_generator_z(std::string id_collision_obj, geometry_msgs::Vector3 position, geometry_msgs::Vector3 orientation, geometry_msgs::Vector3 dimension, std::string solid_type, std::string header_frame_id)
  {
	  geometry_msgs::Vector3 position_new = position;
	  double ori_x = orientation.x;
	  double ori_y = orientation.y;

	  //definition of the centre of the solid
	  double x = position.x + (sin(ori_y) * dimension.z/2);
	  double y = position.y + (sin(ori_x) * dimension.z/2);
	  double z = position.z + (cos(ori_x) * cos(ori_y) * dimension.z/2);
	  position_new.x = x; position_new.y = y; position_new.z = z;

	  moveit_msgs::CollisionObject collision_object = collision_obj_generator(id_collision_obj, position_new, orientation, dimension, solid_type, header_frame_id);

   	  return collision_object;
  }
  */


  //Callback for the end-effector endpoint state (position, twist, wrench) | global variable
  void callback_Ee(const baxter_core_msgs::EndpointState data){
	  gl_end_point_state = data;
	  //to check if the callback function is running uncomment th following line
	  //std::cout << "XYZ.x: " << data.pose.position.x << std::endl;
  }
  baxter_core_msgs::EndpointState getEndPointStateFromTopic(std::string right_left, ros::NodeHandle &nh)
  {
	  baxter_core_msgs::EndpointState eps2return;
	  if(right_left != right_def && right_left != left_def){
		  std::cout << "Warning: insert correctly to which gripper you want to get the current information. ";
		  std::cout << "Insert '" << right_def << "' or '" << left_def << "', otherwise this function will return an empty message by default." << std::endl;
	  }else{
		  std::string topic_str = base_robot_part + right_left + end_point;
		  if(moveit_side_functions::CheckTopicExistence(topic_str)){
			  //initialization of the exit param
			  ros::Subscriber sub = nh.subscribe <baxter_core_msgs::EndpointState>(topic_str, 10, callback_Ee);
			  double curr_time = 0.0; double step_time = std_time;
			  //do not proceed until the topic is not read correctly or the maximum time expired "#define exit_finction_time"
			  while (eps2return.pose.position.x == 0.0 && eps2return.pose.position.y == 0.0 && curr_time < exit_finction_time)
			  {
				  eps2return = gl_end_point_state;
				  //the robot cannot be in the position (0.0,0.0,z) cuz it is inside the body
				  if(eps2return.pose.position.x == 0.0 && eps2return.pose.position.y == 0.0){
					  curr_time += step_time;
					  moveit_side_functions::standardSleep(step_time);
					  ros::spinOnce();
				  }
			  }
			  //reinitialization of the global variable
			  baxter_core_msgs::EndpointState default_gl_end;
			  gl_end_point_state = default_gl_end;
		  }else{
			  std::cout << "Warning: the topic '" << topic_str << "' does not exist! Probably you are using another topic definition or the robot is still not publishing. ";
			  std::cout << "Check how is the topic defined in you robot and change the topic definition in the #define part of the header file." << std::endl;
			  std::cout << "This function returned an empty message by default." << std::endl;
		  }
	  }

	  return eps2return;
  }

  geometry_msgs::Pose getEePoseFromTopic(std::string right_left, ros::NodeHandle &nh)
  {
	  baxter_core_msgs::EndpointState eps2return = getEndPointStateFromTopic(right_left,nh);
	  return eps2return.pose;
  }
  geometry_msgs::Twist getEeTwistFromTopic(std::string right_left, ros::NodeHandle &nh)
  {
  	  baxter_core_msgs::EndpointState eps2return = getEndPointStateFromTopic(right_left,nh);
  	  return eps2return.twist;
  }
  geometry_msgs::Wrench getEeWrenchFromTopic(std::string right_left, ros::NodeHandle &nh)
  {
  	  baxter_core_msgs::EndpointState eps2return = getEndPointStateFromTopic(right_left,nh);
  	  return eps2return.wrench;
  }

  //Callback for the joints state (position, velocity, effort) | global variable
  void callback_Joint(sensor_msgs::JointState data){
	  gl_joints_state = data;}
  sensor_msgs::JointState getBothArmJointValFromTopic(std::string right_left, ros::NodeHandle &nh)
  {
	  sensor_msgs::JointState joints_val2return;
	  if(right_left != right_def && right_left != left_def){
		  std::cout << "Warning: insert correctly from which arm you want to get the current joint values.";
		  std::cout << "Insert '" << right_def << "' or '" << left_def << "', otherwise this function will return an empty message by default." << std::endl;
	  }else{
		  std::string topic_str = joint_state;
		  if(moveit_side_functions::CheckTopicExistence(topic_str)){
			  //initialization of the exit param
			  double curr_time = 0.0; double step_time = std_time;
			  double control_param = 3000.0; joints_val2return.position.push_back(control_param);
			  //do not proceed until the topic is not read correctly or the maximum time expired "#define exit_finction_time"
			  ros::Subscriber sub = nh.subscribe(topic_str, 10, callback_Joint);
			  while (joints_val2return.position[0] == control_param && curr_time < exit_finction_time)
			  {
				  joints_val2return = gl_joints_state;
				  curr_time += step_time;
				  //the robot joint rotation value cannot be high as control_param
				  if(joints_val2return.position[0] == control_param){
					  curr_time += step_time;
					  moveit_side_functions::standardSleep(step_time);
					  ros::spinOnce();
				  }
			  }
			  // restore the original wrong value
			  if(joints_val2return.position[0] == control_param){
				  joints_val2return.position[0] = 0;}

			  //reinitialization of the global variable
			  sensor_msgs::JointState default_gl_joints;
			  gl_joints_state = default_gl_joints;
		  }else{
			  std::cout << "Warning: the topic '" << topic_str << "' does not exist! Probably you are using another topic definition or the robot is still not publishing.";
			  std::cout << "Check how is the topic defined in you robot and change the topic definition in the #define part of the header file." << std::endl;
			  std::cout << "This function returned an empty message by default." << std::endl;
		  }
	  }

	  return joints_val2return;
  }

  std::vector<double> getOneArmJointPositionFromTopic(std::string right_left, ros::NodeHandle &nh)
  {
	  std::vector<double> joints_val2return;
	  std::string temp_str;
	  //selection of the joint I need, sorting them
	  std::vector<std::string> joints_name = joint_names();
	  int joints_num = joints_name.size();
	  joints_val2return.resize(joints_num);

	  sensor_msgs::JointState local_joints_state = getBothArmJointValFromTopic(right_left, nh);

	  for(int i = 0; i < joints_num; i++)
	  {
		  temp_str = right_left + joints_name[i];
		  for(int y = 0; y < local_joints_state.name.size(); y++)
		  {
			  if(local_joints_state.name[y] == temp_str){
				  joints_val2return[i] = local_joints_state.position[y];
				  break;
			  }
		  }
	  }

	  return joints_val2return;
  }
  std::vector<double> getOneArmJointVelocityFromTopic(std::string right_left, ros::NodeHandle &nh)
  {
	  std::vector<double> joints_val2return;
	  std::string temp_str;
	  //selection of the joint I need, sorting them
	  std::vector<std::string> joints_name = joint_names();
	  int joints_num = joints_name.size();
	  joints_val2return.resize(joints_num);

	  sensor_msgs::JointState local_joints_state = getBothArmJointValFromTopic(right_left, nh);

	  for(int i = 0; i < joints_num; i++)
	  {
		  temp_str = right_left + joints_name[i];
		  for(int y = 0; y < local_joints_state.name.size(); y++)
		  {
			  if(local_joints_state.name[y] == temp_str){
				  joints_val2return[i] = local_joints_state.velocity[y];
				  break;
			  }
		  }
	  }

	  return joints_val2return;
  }
  std::vector<double> getOneArmJointEffortFromTopic(std::string right_left, ros::NodeHandle &nh)
  {
	  std::vector<double> joints_val2return;
	  std::string temp_str;
	  //selection of the joint I need, sorting them
	  std::vector<std::string> joints_name = joint_names();
	  int joints_num = joints_name.size();
	  joints_val2return.resize(joints_num);

	  sensor_msgs::JointState local_joints_state = getBothArmJointValFromTopic(right_left, nh);

	  for(int i = 0; i < joints_num; i++)
	  {
		  temp_str = right_left + joints_name[i];
		  for(int y = 0; y < local_joints_state.name.size(); y++)
		  {
			  if(local_joints_state.name[y] == temp_str){
				  joints_val2return[i] = local_joints_state.effort[y];
				  break;
			  }
		  }
	  }

	  return joints_val2return;
  }

  std::vector<double> getBothArmJointPositionFromTopic(ros::NodeHandle &nh)
  {
	  std::vector<double> joints_val2return;
	  std::vector<double> left_temp = getOneArmJointPositionFromTopic(left_def,nh);
	  std::vector<double> right_temp = getOneArmJointPositionFromTopic(right_def,nh);

	  joints_val2return = moveit_side_functions::vector_two_cluster(left_temp, right_temp);

	  return joints_val2return;
  }
  std::vector<double> getBothArmJointVelocityFromTopic(ros::NodeHandle &nh)
  {
	  std::vector<double> joints_val2return;
	  std::vector<double> left_temp = getOneArmJointVelocityFromTopic(left_def,nh);
	  std::vector<double> right_temp = getOneArmJointVelocityFromTopic(right_def,nh);

	  joints_val2return = moveit_side_functions::vector_two_cluster(left_temp, right_temp);

	  return joints_val2return;
  }
  std::vector<double> getBothArmJointEffortFromTopic(ros::NodeHandle &nh)
  {
	  std::vector<double> joints_val2return;
	  std::vector<double> left_temp = getOneArmJointEffortFromTopic(left_def,nh);
	  std::vector<double> right_temp = getOneArmJointEffortFromTopic(right_def,nh);

	  joints_val2return = moveit_side_functions::vector_two_cluster(left_temp, right_temp);

	  return joints_val2return;
  }



// End namespace "moveit_basics_functions"
}


namespace obj_functions
{
  ////// ROBOT FUNCTIONS //////
  std::vector<std::string> GroupNameAvailable(void)
  {
	  std::vector<std::string> names;
	  names.resize(3);
	  names[0] = MAC_SUM(left_def, arm_group_name);
	  names[1] = MAC_SUM(left_def, arm_group_name);
	  // the option "both_arms" must be always the last one
	  names[2] = both_arms_group_name;

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
		  std::cout << "Warning: The planner chosen do not exit in OMPL Default. Call the function 'moveit_basics_functions::getOmplPlannerList() to check the available planners." << std::endl;
	      std::cout << "Warning: The planner for the group " << obj_group << " is still the last one successfully set." << std::endl;
     }

	 return success;
  }

  //YES effective check
  bool setPlanningTime(moveit::planning_interface::MoveGroup& obj, double new_time)
  {
	  bool success = false;
	  if (new_time < 0.0){
		  std::cout << "Warning: The planning time cannot be negative! Previous value is maintained." << std::endl;
	  }else{

		  //effective setting check
		  int exit_count = 0;
		  while (!success && exit_count <= moveit_side_functions::_get_attempt_val()){
			  obj.setPlanningTime(new_time);
			  moveit_side_functions::standardSleep();
			  if(obj.getPlanningTime() == new_time){
				  success = true;
				  std::cout << "Done correctly: The new planning time of the group " << obj.getName() << " is " << new_time << "." << std::endl;
			  }else{
				  exit_count++;
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

  //YES effective check
  bool setEePoseTarget(moveit::planning_interface::MoveGroup& obj, geometry_msgs::Pose pose, std::string left_right)
  {
	  bool success = false;
	  std::vector<std::string> pos_groups = GroupNameAvailable();
	  std::string gripper;

	  if(obj.getName() == pos_groups[pos_groups.size() -1] && (left_right != right_def || left_right != left_def)){
		  std::cout << "Warning: insert correctly to which gripper you want to assign to that pose msg. ";
		  std::cout << "Insert '" << right_def << "' or '" << left_def << "', otherwise this function will not return positive value." << std::endl;
		  return false;
	  }else if(obj.getName() == pos_groups[pos_groups.size() -1] && (left_right == right_def || left_right == left_def)){
		  gripper = left_right + gripper_def;
	  }else if(obj.getName() != pos_groups[pos_groups.size() -1]){
		  gripper = obj.getEndEffectorLink();
	  }

	  //effective setting check
	  int exit_count = 0;
	  while (!success && exit_count <= moveit_side_functions::_get_attempt_val()){
		  obj.setPoseTarget(pose, gripper);
	  	  moveit_side_functions::standardSleep();
	  	  geometry_msgs::Pose goal_pose = moveit_side_functions::PoseStamped2Pose(obj.getPoseTarget(gripper));
	  	  if(moveit_side_functions::PoseEquivalence(goal_pose, pose, 2)){
	  		  success = true;
	  		  std::cout << "Done correctly: The goal pose time of the " << gripper << " of the group " << obj.getName() << " has been set." << std::endl;
	  	  }else{
	  		  exit_count++;
	  	  }
	  }

	  return success;
  }

  //--- effective check
  bool setJointValuesTarget(moveit::planning_interface::MoveGroup& obj, std::vector<double> vector, std::string r_l_single)
  {
	  // All the possible groups
	  std::vector<std::string> pos_groups = GroupNameAvailable();
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
		  if(r_l_single != right_def || r_l_single != left_def)
			  return false;
		  else if(r_l_single == right_def || r_l_single == left_def)
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

  //YES effective check
  bool setJointTolerance(moveit::planning_interface::MoveGroup& obj, double toll)
  {
	  bool success = false;
	  //positive conversion
	  if(toll < 0.0){
		  std::cout << "Warning: the tolerance value has been converted to positive one." << std::endl;
		  toll = moveit_side_functions::abs_f(toll);
	  }
	  //degree to radiant conversion
	  double toll_rad = moveit_side_functions::deg2rad(toll);

	  //effective setting check
	  int exit_count = 0;
	  while(!success && exit_count <= moveit_side_functions::_get_attempt_val())
	  {
		  obj.setGoalJointTolerance(toll_rad);
		  moveit_side_functions::standardSleep();
		  if(obj.getGoalJointTolerance() == toll_rad){
			  success = true;
			  std::cout << "Done correctly: The new joint tolerance of the group " << obj.getName() << " is " << toll << " degree." << std::endl;
		  }else{
			  exit_count++;
		  }
	  }

	  return success;
  }

  //YES effective check
  bool setEeOrientationTolerance(moveit::planning_interface::MoveGroup& obj, double toll)
  {
	  bool success = false;
	  //positive conversion
	  if(toll < 0.0){
		  std::cout << "Warning: the tolerance value has been converted to positive one." << std::endl;
		  toll = moveit_side_functions::abs_f(toll);
	  }
	  //degree to radiant conversion
	  double toll_rad = moveit_side_functions::deg2rad(toll);

	  //effective setting check
	  int exit_count = 0;
	  while(!success && exit_count <= moveit_side_functions::_get_attempt_val())
	  {
		  obj.setGoalOrientationTolerance(toll_rad);
		  moveit_side_functions::standardSleep();
		  if(obj.getGoalOrientationTolerance() == toll_rad){
			  success = true;
			  std::cout << "Done correctly: The new end-effector orientation tolerance of the group " << obj.getName() << " is " << toll << " degree." << std::endl;
		  }else{
			  exit_count++;
		  }
	  }

	  return success;
  }

  //YES effective check
  bool setEePositionTolerance(moveit::planning_interface::MoveGroup& obj, double toll)
  {
	  bool success = false;
	  //positive conversion
	  if(toll < 0.0){
		  std::cout << "Warning: the tolerance value has been converted to positive one." << std::endl;
		  toll = moveit_side_functions::abs_f(toll);
	  }

	  //effective setting check
	  int exit_count = 0;
	  while(!success && exit_count <= moveit_side_functions::_get_attempt_val())
	  {
		  obj.setGoalPositionTolerance(toll);
		  moveit_side_functions::standardSleep();
		  if(obj.getGoalPositionTolerance() == toll){
			  success = true;
			  std::cout << "Done correctly: The new end-effector position tolerance of the group " << obj.getName() << " is " << toll << " meter." << std::endl;
		  }else{
			  exit_count++;
		  }
	  }

	  return success;
  }

  //-- effective check
  bool setConstraint(moveit::planning_interface::MoveGroup& obj, moveit_msgs::Constraints constraint)
  {
	  obj.setPathConstraints(constraint);

	  return true;
  }

  //NO effective check
  bool clearConstraints(moveit::planning_interface::MoveGroup& obj)
  {
	  obj.clearPathConstraints();

	  return true;
  }

  //YES effective check
  bool plan_execute_safely(moveit::planning_interface::MoveGroup& obj)
  {
	  bool success;
	  moveit::planning_interface::MoveGroup::Plan my_plan;
	  success = obj.plan(my_plan);
	  if(success)
		  obj.execute(my_plan);
	  else
	    std::cout << "Something went wrong during planning!" << std::endl;

	  return success;
  }

  //YES effective check
  bool plan_execute_safely_time_incresing(moveit::planning_interface::MoveGroup& obj, bool restore_initial)
  {
	  bool success = false;
	  double curr_time = obj.getPlanningTime();
	  double initial_curr_time = curr_time;
	  while(!success && curr_time <= time_exit)
	  {
		  success = plan_execute_safely(obj);
		  if (!success){
			  curr_time += 2.0;
			  setPlanningTime(obj, curr_time);
		  }
	  }
	  //restore the previous value
	  if(initial_curr_time != curr_time && restore_initial)
		  setPlanningTime(obj, initial_curr_time);

	  return success;
  }

  //YES effective check
  bool plan_execute_safely_attempt_incresing(moveit::planning_interface::MoveGroup& obj, bool restore_initial)
  {
	  bool success = false;
	  unsigned int default_att = 1;
	  unsigned int curr_att = default_att;
	  while(!success && curr_att <= att_exit)
	  {
		  success = plan_execute_safely(obj);
		  if (!success){
			  curr_att += 2;
			  setPlanningAttempts(obj, curr_att);
		  }
	  }
	  //restore the previous the default value
	  if(default_att != curr_att && restore_initial)
		  setPlanningAttempts(obj, default_att);

	  return success;
  }

  //NO effective check
  void move_direct(moveit::planning_interface::MoveGroup& obj)
  {
	  obj.move();
  }

  //NO effective check
  void motion_stop(moveit::planning_interface::MoveGroup& obj)
  {
	  obj.stop();
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

  //NO effective check (partial check on the link name)
  bool attachObj2group(moveit::planning_interface::MoveGroup& group, std::string obj_id, std::string link_id,  double time2wait)
  {
	  std::vector<std::string> link_names = getLinkNames(group);
	  for(int i = 0; i < link_names.size(); i++)
	  {
		  if(link_names[i] == link_id){
			  std::cout << "The object " << obj_id << " has been attached to the link " << link_id << " of the group " << group.getName() << " correctly." << std::endl;
			  group.attachObject(obj_id, link_id);
			  break;
		  }else if(i == link_names.size() -1 && link_names[i] == generic_str){
			  std::cout << "The object " << obj_id << " has been attached to the default link " << group.getEndEffectorLink() << " of the group " << group.getName() << " correctly." << std::endl;
			  group.attachObject(obj_id);
			  break;
		  }else if(i == link_names.size() -1 && link_names[i] != generic_str){
			  std::cout << "Error: The object " << obj_id << " has been attached to the default link " << group.getEndEffectorLink() << " of the group " << group.getName() << " because the link " << link_id << " does not exist!" << std::endl;
			  std::cout << "Call the function 'obj_function::getLinkNames(" << group.getName() << ") to check the available link names." << std::endl;
			  return false;
		  }
	  }
	  moveit_side_functions::standardSleep(time2wait);

	  return true;
  }

  //NO effective check
  bool detachObj2group(moveit::planning_interface::MoveGroup& group, std::string obj_id, double time2wait)
  {
  	  group.detachObject(obj_id);
	  moveit_side_functions::standardSleep(time2wait);

  	  return true;
  }

    /* MOVEIT ERROR
    std::vector<double> getCurrentArmJointValues(moveit::planning_interface::MoveGroup& obj, std::string r_l_single)
    {
  	  std::vector<std::string> pos_groups = GroupNameAvailable();
  	  std::vector<std::string> joint_names = getJointNames(obj);
  	  std::vector<double> joint_val;

  	  //if two arms and I just want to know one of them
  	  if(obj.getName() == pos_groups[pos_groups.size() -1] && (r_l_single == right_def || r_l_single == left_def)){
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
  	  std::vector<std::string> pos_groups = GroupNameAvailable();
    	  std::string gripper;
    	  if(obj.getName() == pos_groups[pos_groups.size() -1] && (left_right != right_def || left_right != left_def)){
    		  std::cout << "Warning: insert correctly to which gripper you want to get the current pose.";
    		  std::cout << "Insert '" << right_def << "' or '" << left_def << "', otherwise this function will not return the right one by default." << std::endl;
    		  left_right = right_def;
    		  gripper = left_right + gripper_def;
    	  }else if(obj.getName() == pos_groups[pos_groups.size() -1] && (left_right == right_def || left_right == left_def)){
    		  gripper = left_right + gripper_def;
    	  }else if(obj.getName() != pos_groups[pos_groups.size() -1]){
    		  gripper = obj.getEndEffectorLink();
    	  }

    	  geometry_msgs::PoseStamped poseStamp2return = obj.getCurrentPose(gripper);
    	  geometry_msgs::Pose pose2return = moveit_side_functions::PoseStamped2Pose(poseStamp2return);
  	  return pose2return;
    }

    geometry_msgs::Quaternion getCurrentGripperQuaternion(moveit::planning_interface::MoveGroup& obj, std::string left_right)
    {
  	  geometry_msgs::Pose temp_pose = getCurrentGripperPose(obj, left_right);
  	  geometry_msgs::Quaternion quat2return = temp_pose.orientation;
  	  return quat2return;
    }

    geometry_msgs::Vector3 getCurrentGripperRPY(moveit::planning_interface::MoveGroup& obj, std::string left_right)
    {
      std::vector<std::string> pos_groups = GroupNameAvailable();
      std::string gripper;
      if(obj.getName() == pos_groups[pos_groups.size() -1] && (left_right != right_def || left_right != left_def)){
    	  std::cout << "Warning: insert correctly to which gripper you want to get the current pose. ";
    	  std::cout << "Insert '" << right_def << "' or '" << left_def << "', otherwise this function will not return the right one by default." << std::endl;
    	  left_right = right_def;
    	  gripper = left_right + gripper_def;
      }else if(obj.getName() == pos_groups[pos_groups.size() -1] && (left_right == right_def || left_right == left_def)){
    	  gripper = left_right + gripper_def;
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
    */



// End namespace "obj_functions"
}

