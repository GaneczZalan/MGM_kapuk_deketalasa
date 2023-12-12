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

#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"
#include <visualization_msgs/Marker.h>

move_base_msgs::MoveBaseGoal goal;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
ros::Subscriber sub_points;
ros::Subscriber odom_sub;
void points_callback(const pcl::PointCloud<pcl::PointXYZ>& inputCloud);
void NodeHandle(ros::NodeHandle nh);
pcl::PointXYZ referencePoint;
ros::Publisher marker_pub;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void findHalfwayPoint(const pcl::PointCloud<pcl::PointXYZ>& inputCloud, Eigen::Vector3f& halfwayPoint)
{
    // Check if the input cloud is empty
    if (inputCloud.empty())
    {
        std::cerr << "Input cloud is empty. Cannot find halfway point." << std::endl;
        return;
    }

    // Initialize variables to store the indices of the two closest points
    int index1 = -1;
    int index2 = -1;
    double minDistance1 = 0;
    double minDistance2 = 0;

    // Iterate through all points in the input cloud
    for (size_t i = 0; i < inputCloud.size(); ++i)
    {
        pcl::PointXYZ temp_point;
        temp_point.x=inputCloud[i].x;
        temp_point.y=inputCloud[i].y;
        temp_point.z=inputCloud[i].z;
        // Calculate the Euclidean distance between the current point and the reference point
        double distance = pcl::euclideanDistance(temp_point, referencePoint);

        // Update the indices of the two closest points
        if (distance < minDistance1)
        {
            minDistance2 = minDistance1;
            index2 = index1;

            minDistance1 = distance;
            index1 = static_cast<int>(i);
        }
        else if (distance < minDistance2)
        {
            minDistance2 = distance;
            index2 = static_cast<int>(i);
        }
    }

    // Check if two closest points were found
    if (index1 == -1 && index2 == -1)
    {
        ROS_INFO("nincs kozeppont");
        return;
    }

    // Calculate the halfway point between the two closest points
    halfwayPoint = 0.5 * (inputCloud[index1].getVector3fMap() + inputCloud[index2].getVector3fMap());

    visualization_msgs::Marker circle_marker;
            circle_marker.header.frame_id = inputCloud.header.frame_id;
            circle_marker.header.stamp = ros::Time::now();
            circle_marker.ns = "circles";
            circle_marker.action = visualization_msgs::Marker::ADD;
            circle_marker.pose.orientation.w = 1.0;
            circle_marker.type = visualization_msgs::Marker::SPHERE;
            circle_marker.scale.x = 0.5;
            circle_marker.scale.y = 0.5;
            circle_marker.scale.z = 0.5;
            circle_marker.color.r = 0.3;
            circle_marker.color.g = 0.75;
            circle_marker.color.b = 0.0;
            circle_marker.color.a = 1.0;
            circle_marker.pose.position.x = halfwayPoint[0];
            circle_marker.pose.position.y = halfwayPoint[1];
            circle_marker.pose.position.z = halfwayPoint[2];
            marker_pub.publish(circle_marker);
}

void moveToHalfwayPoint(const Eigen::Vector3f& halfwayPoint)
{
    MoveBaseClient ac("move_base", true);
    
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = halfwayPoint[0];
    goal.target_pose.pose.position.y = halfwayPoint[1];
    goal.target_pose.pose.position.z = 0;

    goal.target_pose.pose.orientation.x = 0.0;
    goal.target_pose.pose.orientation.y = 0.0;
    goal.target_pose.pose.orientation.z = 0.0;
    goal.target_pose.pose.orientation.w = 1.0;

    ROS_INFO("Sending goal");
    ac.sendGoal(goal);

    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Hooray, the robot reached the goal!");
    }
    else
    {
        ROS_INFO("The robot failed to reach the goal for some reason");
    }
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // Extract the position information from the odometry message
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double z = msg->pose.pose.position.z;

    // Update the reference point
    referencePoint.x = x;
    referencePoint.y = y;
    referencePoint.z = z;
}

void points_callback(const pcl::PointCloud<pcl::PointXYZ>& inputCloud)
{
    Eigen::Vector3f halfwayPoint;
    findHalfwayPoint(inputCloud, halfwayPoint);
    moveToHalfwayPoint(halfwayPoint);
}

void NodeHandle(ros::NodeHandle nh)
{
    ros::Subscriber sub_points = nh.subscribe("filtered_cloud", 1, points_callback);
    ros::Subscriber odom_sub = nh.subscribe("odom", 1, odomCallback);
    marker_pub=nh.advertise<visualization_msgs::Marker>("midpoint",1);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "map");

    ros::NodeHandle nh;
    NodeHandle(nh);

    ros::spin();
    return 0;
}



/*class turtle_bot_control 
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
}*/