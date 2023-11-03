#include "ros/ros.h"

#include "std_msgs/String.h"



void callback_string(const std_msgs::String::ConstPtr& msg){
	
	// ROS_INFO("I heard: [%s]", msg->data.c_str());
	ROS_INFO_STREAM("Receiving... " << msg->data.c_str());

}




int main(int argc, char **argv){

	ros::init(argc, argv, "test_node_1");

	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("/topic1", 1000, callback_string);

	ros::Publisher pub = n.advertise<std_msgs::String>("/topic1", 1000);

	ros::Rate r(1); 


    while(ros::ok()){

        std_msgs::String msg;
        msg.data = "hello";
        pub.publish(msg);
		ROS_INFO("Sending...");
		ros::spinOnce();
        r.sleep();
		
    }


}