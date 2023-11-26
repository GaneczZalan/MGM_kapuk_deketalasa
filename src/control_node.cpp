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
#include "turtlesim/Pose.h"
#include <nav_msgs/Odometry.h>
#include <tf2/utils.h>
#include <geometry_msgs/Quaternion.h>

class turtle_bot_control 
{

    public:
        turtle_bot_control(): nh("~")
        {
            cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
            pose_sub = nh.subscribe("/pose", 1, &turtle_bot_control::turtle_pose_callback, this);

            goal_x = -1.5;
            goal_y = 1.5;

            loop_rate = 10;
            ros::Rate rate(loop_rate);
            
            while (1)
            {
                double angle = atan2(goal_y - current_y, goal_x - current_x);
                nav_msgs::Odometry odometry;
                
                
                double yaw = tf2::getYaw(odometry.pose.pose.orientation);
                if(abs(yaw-angle)>0.5)
                {
                    control_turtle(0.05, 0.2);
                }
                else control_turtle(0.05, 0);
                

                rate.sleep();

                ros::spinOnce();
            }
        }

    private:
        ros::NodeHandle nh;
        ros::Publisher cmd_vel_pub;
        ros::Subscriber pose_sub;

        double current_x;
        double current_y;

        double goal_x;
        double goal_y
}








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