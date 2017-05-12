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

  geometry_msgs::Pose makePose(geometry_msgs::Vector3 RPY_orientation, geometry_msgs::Vector3 XYZ_location)
  {
    geometry_msgs::Pose pose;
  	pose.orientation = moveit_side_functions::RPY2Quat(RPY_orientation);
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


namespace moveit_basics_bax
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
	  geometry_msgs::Vector3 RPY_up_x = moveit_basics_bax::vertical_orientation_x();
	  geometry_msgs::Vector3 RPY_up_y = moveit_basics_bax::vertical_orientation_y();
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

  moveit_msgs::OrientationConstraint orient_constr_definition(geometry_msgs::Vector3 RPY, std::string link_name, float toll_x, float toll_y, float toll_z, float weight, std::string header_frame_id)
  {
      moveit_msgs::OrientationConstraint orient_constr;
  	  orient_constr.orientation = moveit_side_functions::RPY2Quat(RPY);
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

// End namespace "moveit_basics_bax"
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

  bool setPlanner(moveit::planning_interface::MoveGroup& obj, std::string new_planner)
  {
	  bool success = false;
	  std::string obj_group = obj.getName();
	  std::string planner_ID = obj.getDefaultPlannerId(obj_group);
	  std::vector<std::string> planner_list = moveit_basics_bax::getOmplPlannerList();
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
		  std::cout << "Warning: The planner chosen do not exit in OMPL Default. Call the function 'moveit_basics_bax::getOmplPlannerList() to check the planners available." << std::endl <<
	      std::cout << "Warning: The new planner for the group " << obj_group << " is still " << planner_ID << "." << std::endl;
     }

	 return success;
  }

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

  void move_f(moveit::planning_interface::MoveGroup& obj)
  {
	  obj.move();
  }




  ////// ENVIRONMENT FUNCTIONS //////




// End namespace "obj_functions"
}

