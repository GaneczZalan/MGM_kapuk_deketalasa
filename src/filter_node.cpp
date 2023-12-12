#include "ros/ros.h"
#include <iostream>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/centroid.h>
#include <visualization_msgs/MarkerArray.h>

ros::Subscriber cloud_sub;
ros::Publisher marker_pub;
ros::Publisher filtered_cloud_pub;

void detectCones(const sensor_msgs::PointCloud2ConstPtr& inputCloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*inputCloud, *outputCloud);

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setModelType(pcl::SACMODEL_CIRCLE2D);
    seg.setDistanceThreshold(0.05);
    seg.setMaxIterations(1000);

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> circles;
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    while (true)
    {
        seg.setInputCloud(outputCloud);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.size() == 0)
        {
            break;
        }
        else
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr circleCloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(outputCloud);
            extract.setIndices(inliers);
            extract.setNegative(false);
            extract.filter(*circleCloud);

            Eigen::Vector4f center;
            center[0] = coefficients->values[0];
            center[1] = coefficients->values[1];
            center[2] = 0;

            // Now 'center' contains the middle point of the detected circle

            circles.push_back(circleCloud);

            extract.setNegative(true);
            extract.filter(*outputCloud);
        }
    }
    pcl::PointCloud<pcl::PointXYZ> localCloud;
    for (size_t i = 0; i < circles.size(); ++i)
    {
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*circles[i], centroid);

        pcl::PointXYZ newPoint;
        newPoint.x = centroid[0];
        newPoint.y = centroid[1];
        newPoint.z = 0;

        localCloud.push_back(newPoint);
    }
    localCloud.header.frame_id=inputCloud->header.frame_id;
    localCloud.header.seq=inputCloud->header.seq;
    // Publish the vector of circle centers

    visualization_msgs::MarkerArray marker_array;
    for (size_t i = 0; i < circles.size(); ++i)
    {

            visualization_msgs::Marker circle_marker;
            circle_marker.header.frame_id = inputCloud->header.frame_id;
            circle_marker.header.stamp = ros::Time::now();
            circle_marker.ns = "circles";
            circle_marker.action = visualization_msgs::Marker::ADD;
            circle_marker.pose.orientation.w = 1.0;
            circle_marker.id = i;
            circle_marker.type = visualization_msgs::Marker::SPHERE;
            circle_marker.scale.x = 0.1;
            circle_marker.scale.y = 0.1;
            circle_marker.scale.z = 0.1;
            circle_marker.color.r = 0.0;
            circle_marker.color.g = 1.0;
            circle_marker.color.b = 0.0;
            circle_marker.color.a = 1.0;
            circle_marker.pose.position.x = localCloud[i].x;
            circle_marker.pose.position.y = localCloud[i].y;
            circle_marker.pose.position.z = 0;

            marker_array.markers.push_back(circle_marker);
    }

    marker_pub.publish(marker_array);
    filtered_cloud_pub.publish(localCloud);
}

void node_handle(ros::NodeHandle& nh)
{
    cloud_sub = nh.subscribe("cloud", 1, detectCones);
    marker_pub = nh.advertise<visualization_msgs::MarkerArray>("circle_markers", 1);
    filtered_cloud_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("filtered_cloud", 1);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "map");
    ros::NodeHandle nh;
    node_handle(nh);
    ros::spin();
    return 0;
}
