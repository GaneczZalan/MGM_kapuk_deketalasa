#include "ros/ros.h"
#include <iostream>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>
#include "geometry_msgs/Twist.h"

#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl/common/transforms.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>

int main(int a, char** aa) {

    ros::init(a, aa, "control");

    ros::NodeHandle nh;

    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    ros::Rate loop_rate(10);

    double steering_angle = 0.2;

    while (ros::ok()) {
        // Create a Twist message
        geometry_msgs::Twist cmd_vel_msg;

        // Set linear velocity (constant speed)
        cmd_vel_msg.linear.x = 0.2; // Adjust the speed as needed

        // Set angular velocity based on the specified steering angle
        cmd_vel_msg.angular.z = steering_angle;

        // Publish the Twist message
        cmd_vel_pub.publish(cmd_vel_msg);

        // Sleep to maintain the loop rate
        loop_rate.sleep();
        
        // Spin once to trigger the callbacks
        ros::spinOnce();

        
    }

    return 0;
}