#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_listener.h>
#include <gazebo_msgs/ModelStates.h>

class OdomBaseBroadcaster
{
private:
	// Declare the Node Handle to be used in from all the class
	ros::NodeHandle node;

    // Declare a broadcaster
	tf::TransformBroadcaster br;

	// Declare a listener of transforms
	tf::TransformListener listener;

	// Declare a suscriber to recieve the gazebo information
	ros::Subscriber sub;

	// Declare a variable to store the pose recieved from Gazebo
	geometry_msgs::Pose pose_gazeboWorld;

	// Declare a variable to store the transform between gazebo_world & map(odom)
	tf::StampedTransform map_gazeboWorld_transform;

	// Create a flag to know if the first pose have been recieved
	bool FirstPoseRecieved = false;

	// Declare the parameters to be recieved
    std::string robot_model_name;
    int robot_model_index;

    void getIndex(const gazebo_msgs::ModelStates::ConstPtr& msg, int array_size)
    {
    	int index = 0;

    	while(index < array_size){
    		if(msg->name[index] == this->robot_model_name){
    			this->robot_model_index = index;
    			return; 
    		}
    		index++;
    	}

    	ROS_ERROR("Odom-baselink Broadcaster: Model name of the robot not present in the gazebo simulation");
    }

public:
    void infoGazebo(const gazebo_msgs::ModelStates::ConstPtr& msg)
    {
    	if(this->robot_model_index == -1)
    	{
    		int array_size = msg->name.size();
    		getIndex(msg, array_size);
    	}

		// Get the information of the pose of the robot from the gazebo_msg
		this->pose_gazeboWorld = msg->pose[this->robot_model_index];
		this->FirstPoseRecieved = true;
    }

    void wait_for_first_pose(){
    	while(!this->FirstPoseRecieved){
    		ros::spinOnce();
    	}
    }

    void get_gazebo_map_transform(){
    	// Get the transform from the map to the gazeboWorld
    	/* 
    	Explanation (THIS IS A HELL OF AN API): 
    		1- In waitForTransform() and lookupTransform() the first argument is the target frame and the second the source frame.
    		2- But this will return the transform to transform the points in the source frame to the target frame, what is the inverse
    		of the transformation from the source frame to the target frame.
			3- As we are interested in the transform between frames we will ask directly for the invers transformation, asking directly
			for the transform from the target to the source frame.

			In particular:
			lookupTransform("/map", "/gazebo_world") will return the transformation to transform point in /gazebo_world frame 
			to /map frame, so it is the invers of the transformation of the /gazebo_world frame to the /map frame, 
			which the transformation between /map and /gazebo_world frame (What we want!) 
			
    	*/
		this->listener.waitForTransform("/map", "/gazebo_world", ros::Time(0), ros::Duration(10.0));
		this->listener.lookupTransform("/map", "/gazebo_world", ros::Time(0), this->map_gazeboWorld_transform);
    }

    void transformBroadcasting(){

    	// Create the current transform from the gazebo_world to the base_link
		tf::Transform gazeboWorld_baselink_transform;

		gazeboWorld_baselink_transform.setOrigin( 
		tf::Vector3(this->pose_gazeboWorld.position.x, 
		this->pose_gazeboWorld.position.y, 
		this->pose_gazeboWorld.position.z) 
		);

		tf::Quaternion q(
			this->pose_gazeboWorld.orientation.x,
			this->pose_gazeboWorld.orientation.y,
			this->pose_gazeboWorld.orientation.z,
			this->pose_gazeboWorld.orientation.w
			);
		gazeboWorld_baselink_transform.setRotation(q);

		// The transform from odom (odom-map and gazebo_world-map transforms should be published) to base_link 
		//will be the multiplication of the two above
		tf::Transform odom_baselink_transform;
		odom_baselink_transform = this->map_gazeboWorld_transform * gazeboWorld_baselink_transform;

		// Broadcast the calculated transform
		br.sendTransform(tf::StampedTransform(odom_baselink_transform, ros::Time::now(), "/odom", "/base_link"));
    }

    void delay(float time){
    	//Sleep in order to give time to spawn
    	ros::Duration(time).sleep(); 
    }

    OdomBaseBroadcaster(){

    	// Create a Node Handler
		this->node = ros::NodeHandle("~");

        this->node.param<std::string>("robot_model_name", this->robot_model_name, "/");
        this->node.param<int>("robot_model_index", this->robot_model_index, -1);

    }

    void run(){
		
		// Set a loop_rate to publish the transforms
        ros::Rate loop_rate(10);

        // Subscribe to the gazebo topic to recieve the initial pose of the robot
        this->sub = this->node.subscribe("/gazebo/model_states", 1000, &OdomBaseBroadcaster::infoGazebo, this);

        wait_for_first_pose();

       	get_gazebo_map_transform();

        while (ros::ok())
        {
            // Broadcast the transform
        	transformBroadcasting();

            // And throttle the loop
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

};


int main(int argc, char **argv){
    // Initialize ROS
    ros::init(argc, argv, "odom_baselink_broadcaster"); 

    // Create the broadcaster object Run it
    OdomBaseBroadcaster broadcaster;

    // Delay to let the simulation start
    broadcaster.delay(5.0);

    // Run it
    broadcaster.run();

    return 0;
}