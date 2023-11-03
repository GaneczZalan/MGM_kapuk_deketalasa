#include "ros/ros.h"

#include "std_msgs/String.h"



void callback_string(const std_msgs::String::ConstPtr& msg){
	
	ROS_WARN("I heard: [%s]", msg->data.c_str());

}




int main(int argc, char **argv){

	ros::init(argc, argv, "test_node_2");

	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("/topic1", 1000, callback_string);

	ros::spin();


}