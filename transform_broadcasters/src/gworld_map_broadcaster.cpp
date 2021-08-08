#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <gazebo_msgs/GetModelState.h>


int main(int argc, char** argv){
	// ROS initialization
	ros::init(argc, argv, "gworld_map_broadcaster");

	// Create a Node Handle
	ros::NodeHandle node("~");

	// Parameters used
	std::string robot_model_name;
	std::string world_model_name;

	node.param<std::string>("robot_model_name", robot_model_name, "/");
	node.param<std::string>("world_model_name", world_model_name, "agriculture_geom");

	//ROS_INFO("%s", robot_model_name.c_str());

	// Initialize a ros::Rate object to publish at a certain frequency
	ros::Rate loop_rate(100); 

	// Create a broadcaster
	static tf::TransformBroadcaster br;

	// Get the first position of the robot
	ros::ServiceClient client = node.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
	geometry_msgs::Pose initial_pose;

	gazebo_msgs::GetModelState srv;
	srv.request.model_name = robot_model_name;
	srv.request.relative_entity_name = world_model_name;

	ros::Duration(5.0).sleep(); //Sleep in order to give time to spawn
	client.waitForExistence( ros::Duration(10)); 
	if(client.call(srv)){
		initial_pose = srv.response.pose;
	}else{
		ROS_ERROR("Failed to call service GetModelState");
    	return 1;
	}

	// Create the transform to be broadcasted 
	tf::Transform gazeboWorld_map_transform;
	gazeboWorld_map_transform.setOrigin( tf::Vector3(
		initial_pose.position.x, 
		initial_pose.position.y, 
		initial_pose.position.z) 
	);

  	tf::Quaternion q(0., 0., 0., 1.
  		// initial_pose.orientation.x, 
  		// initial_pose.orientation.y, 
  		// initial_pose.orientation.z, 
  		// initial_pose.orientation.w
  		);

  	gazeboWorld_map_transform.setRotation(q);


	while (ros::ok())
	{

		// Send the transform that was created
		br.sendTransform(tf::StampedTransform(gazeboWorld_map_transform, ros::Time::now(), "/gazebo_world", "/map"));

		// Sleep the necessary time to meet the desidered frequency
		loop_rate.sleep();
	}

	return 0;
};