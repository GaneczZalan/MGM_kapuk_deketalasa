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
                    control_turtle(0, 0);
                }
                else control_turtle(0, 0);
                

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
        double goal_y;

        int loop_rate;

        const double dist_treshold = 0.05;

        void turtle_pose_callback(const nav_msgs::Odometry::ConstPtr& odom_msg)
        {
            current_x = odom_msg->pose.pose.position.x;
            current_y = odom_msg->pose.pose.position.y;
        }

        void control_turtle(double linear_vel, double angular_vel)
        {
            geometry_msgs::Twist cmd_vel_msg;
            cmd_vel_msg.linear.x = linear_vel;
            cmd_vel_msg.angular.z = angular_vel;
            cmd_vel_pub.publish(cmd_vel_msg);
        }

        void stop_turtle()
        {
            control_turtle(0,0);
        }
        
        double calculate_distance(double x1, double y1, double x2, double y2)
        {
            return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
        }
};

int main(int a, char** aa) {

    ros::init(a, aa, "control");

    turtle_bot_control control;

    return 0;
}