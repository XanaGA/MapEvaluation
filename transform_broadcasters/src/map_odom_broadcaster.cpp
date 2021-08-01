#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
	// ROS initialization
	ros::init(argc, argv, "map_odom_broadcaster");

	// Create a Node Handle
	ros::NodeHandle node;

	// Initialize a ros::Rate object to publish at a certain frequency
	ros::Rate loop_rate(10); 

	// Create a broadcaster
	static tf::TransformBroadcaster br;

	//Create the transform to be broadcasted
	tf::Transform map_odom_transform;
	map_odom_transform.setIdentity();

	// spin() to coordinate with other nodes
	//ros::spin();

	while (ros::ok())
	{

		//ROS_INFO("Publishing map-odom transform");

		// Send the transform that was created
		br.sendTransform(tf::StampedTransform(map_odom_transform, ros::Time::now(), "/map", "/odom"));

		// Sleep the necessary time to meet the desidered frequency
		loop_rate.sleep();
	}

	return 0;
};