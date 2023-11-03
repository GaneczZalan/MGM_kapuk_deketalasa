#include "ros/ros.h"
#include <iostream>
#include "std_msgs/String.h"



void callback_string(const std_msgs::String::ConstPtr& msg){
	
	ROS_WARN("I scream: [%s]", msg->data.c_str());

}

int main(int argc, char **argv){

	ros::init(argc, argv, "test_node_3");

	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("/topic1", 1000, callback_string);

	ros::Publisher pub = n.advertise<std_msgs::String>("/topic1", 1000);

	ros::Rate r(1); 
	int i=0;

	while(ros::ok()){
		if(i%5==0)
		{
        std_msgs::String msg;
        msg.data = "5th Aladár";
        pub.publish(msg);
		ROS_INFO("Sending...");
		}
		ros::spinOnce();
        r.sleep();
		i++;
    }

}