/*Author: Emanuele Sansebastiano */

#include <ros/ros.h>
#include <tf/transform_listener.h>
//#include <geometry_msgs/PointStamped.h>
#include <tf2_msgs/TFMessage.h>

#include <moveit_side_pkg/side_functions.h>

//global variables
tf2_msgs::TFMessage gl_TFM;


// If there is a PoseStamped in the vector with the same id_name of the PoseStamped selected, this function will return the vector position.
// Otherwise a negative value will be returned
int PoseStamped_find_id(std::vector<geometry_msgs::PoseStamped> &vector, geometry_msgs::PoseStamped &posStamp2check)
{
	for(int i = 0; i < vector.size(); i++)
	{
		if(!(posStamp2check.header.frame_id.compare(vector[i].header.frame_id) != 0))
			return i;
	}

	return -1;
}
// to this function I pass directly the string name
int PoseStamped_find_id(std::vector<geometry_msgs::PoseStamped> &vector, std::string &posStamp_id)
{
	for(int i = 0; i < vector.size(); i++)
	{
		if(!(posStamp_id.compare(vector[i].header.frame_id) != 0))
			return i;
	}

	return -1;
}

// If there is a tfMessage in the vector with the same header and child names of the tfMessage selected, this function will return the vector position.
// Otherwise a negative value will be returned
int TFMessage_find_id(std::vector<tf2_msgs::TFMessage> &vector, tf2_msgs::TFMessage &TFMsgs2check)
{
	for(int i = 0; i < vector.size(); i++)
	{
		if(!(TFMsgs2check.transforms[0].child_frame_id.compare(vector[i].transforms[0].child_frame_id) != 0 && TFMsgs2check.transforms[0].header.frame_id.compare(vector[i].transforms[0].header.frame_id) != 0))
			return i;
	}

	return -1;
}

//This function check the equivalence of two transformations
bool TFM_transf_Equivalence_theshold(tf2_msgs::TFMessage A, tf2_msgs::TFMessage B, double threshold_XYZ = 0.0, double threshold_Quat = 0.0)
{
	namespace msf = moveit_side_functions;
	geometry_msgs::Pose temp_pose_A = msf::makePose(A.transforms[0].transform.rotation, A.transforms[0].transform.translation);
	geometry_msgs::Pose temp_pose_B = msf::makePose(B.transforms[0].transform.rotation, B.transforms[0].transform.translation);

	return msf::PoseEquivalence_theshold(temp_pose_A, temp_pose_B, threshold_XYZ, threshold_Quat);
}

void callback_TFM(tf2_msgs::TFMessage data)
{	gl_TFM = data;
	//std::cout << gl_TFM.transforms[0].header.frame_id << std::endl;
}
void callback_TFM2(tf2_msgs::TFMessage data)
{	gl_TFM = data;
	std::cout << gl_TFM.transforms[0].child_frame_id << std::endl;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "frame_coord_pub");
	ros::NodeHandle nh("~");

	ros::AsyncSpinner spinner(1);

	spinner.start();

	// variable generation
	tf::TransformListener listener(ros::Duration(10));
	std::vector<geometry_msgs::PoseStamped> pose_stamped_vector;
	geometry_msgs::PoseStamped pose_stamped_temp;
	std::vector<tf2_msgs::TFMessage> TFMsgs2check_vector;
	tf2_msgs::TFMessage TFMsgs2check_temp;
	std::string topic_str;
	int temp_val1, temp_val2, temp_val3;

	pose_stamped_temp.header.frame_id = "world";
	pose_stamped_temp.pose.orientation.w = 1.0;

	pose_stamped_vector.push_back(pose_stamped_temp);

	topic_str = "/tf_static";
	//to be sure the topic has been read correctly
	ros::Subscriber sub_tf = nh.subscribe <tf2_msgs::TFMessage>(topic_str, 10, callback_TFM);
	while(gl_TFM.transforms.size() == 0)
	{	ros::spinOnce();}
	TFMsgs2check_vector.push_back(gl_TFM);

	topic_str = "/tf";
	sub_tf.shutdown();
	ros::Subscriber sub_tf2 = nh.subscribe <tf2_msgs::TFMessage>(topic_str, 10, callback_TFM2);
	while(ros::ok())
	{
		//if the header frame (father frame) is already included in the stored list
		temp_val1 = PoseStamped_find_id(pose_stamped_vector, gl_TFM.transforms[0].header.frame_id);
		if(temp_val1 >= 0)
		{
			//if the transformation not already stored or it is not equal to the previous one
			temp_val2 = TFMessage_find_id(TFMsgs2check_vector, gl_TFM);
			//std::cout << gl_TFM.transforms[0].child_frame_id << std::endl;
			//std::cout << temp_val2 << std::endl;
			if(temp_val2 < 0 || !TFM_transf_Equivalence_theshold(TFMsgs2check_vector[temp_val2], gl_TFM))
			{
				std::cout << "passed3" << std::endl;
				//insert the transformation in the stored list
				if(temp_val2 < 0)
				{
					std::cout << "passed" << std::endl;
					TFMsgs2check_vector.push_back(gl_TFM);
					//make the transformation and store the child frame
				    try{
				    	listener.transformPose(gl_TFM.transforms[0].child_frame_id, pose_stamped_vector[temp_val1], pose_stamped_temp);
				    }catch(tf::TransformException& ex){
				    	ROS_ERROR("Received an exception trying to transform a PoseStamp: %s", ex.what());
				    }
				    pose_stamped_vector.push_back(pose_stamped_temp);
				//update the transformation stored in the list
				}else{
					std::cout << "passed2" << std::endl;
					TFMsgs2check_vector[temp_val2] = gl_TFM;
					//make all the transformations from this to the end
					for(int i = temp_val2; i < TFMsgs2check_vector.size(); i++)
					{
						// finding the pose location of the father frame
						temp_val3 = PoseStamped_find_id(pose_stamped_vector, TFMsgs2check_vector[i].transforms[0].header.frame_id);
						// calculation of the child frame (pose_stamped_temp)
					    try{
					    	listener.transformPose(TFMsgs2check_vector[i].transforms[0].child_frame_id, pose_stamped_vector[temp_val3], pose_stamped_temp);
					    }catch(tf::TransformException& ex){
					    	ROS_ERROR("Received an exception trying to transform a PoseStamp: %s", ex.what());
						}
						//update the children frame in the stored list
						temp_val3 = PoseStamped_find_id(pose_stamped_vector, pose_stamped_temp);
						pose_stamped_vector[temp_val3] = pose_stamped_temp;

					}
				}
			}
		}

		//to update gl_TFM
		ros::spinOnce();

		//std::cout << pose_stamped_vector.size() << std::endl;
	}

	ros::shutdown();
	return 0;
}
