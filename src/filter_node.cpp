#include "ros/ros.h"
#include <iostream>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>

#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl/common/transforms.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>

#include <visualization_msgs/MarkerArray.h>

#include <pcl/segmentation/extract_clusters.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>

pcl::PointCloud<pcl::PointXYZ> outputCloud;

 void detectCones(sensor_msgs::PointCloud2 inputCloud) 
 {
        pcl::fromROSMsg(inputCloud, outputCloud);
        /*pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setModelType(pcl::SACMODEL_CIRCLE3D);
        seg.setDistanceThreshold(4);  // Adjust based on your scenario
        seg.setRadiusLimits(0.1, 0.2);  // Set minimum and maximum radius limits in meters
        seg.setMaxIterations(100000);  // Adjust based on your scenario;


        seg.setInputCloud(outputCloud.makeShared());
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.size() == 0) {
            ROS_INFO("No cone detected.");
            return;
        }

        pcl::PointCloud<pcl::PointXYZ> coneCloud;
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(outputCloud.makeShared());
        extract.setIndices(inliers);
        extract.filter(coneCloud);

        // Now 'coneCloud' contains the points representing the detected cones.
        // You can further process and analyze these points to identify cones.

        // For more accurate cone detection, you may want to use more sophisticated algorithms
        // and consider factors like cone size, orientation, and filtering noise.

        // Publish markers for the detected cones
        /*visualization_msgs::MarkerArray marker_array;
        visualization_msgs::Marker cone_marker;
        cone_marker.header.frame_id = outputCloud.header.frame_id; // Adjust the frame_id if needed
        cone_marker.header.stamp = ros::Time::now();
        cone_marker.ns = "cones";
        cone_marker.action = visualization_msgs::Marker::ADD;
        cone_marker.pose.orientation.w = 1.0;
        cone_marker.type = visualization_msgs::Marker::SPHERE;
        cone_marker.scale.x = 0.2; // Adjust the scale based on your cone size
        cone_marker.scale.y = 0.2;
        cone_marker.scale.z = 0.2;
        cone_marker.color.r = 1.0;
        cone_marker.color.g = 0.0;
        cone_marker.color.b = 0.0;
        cone_marker.color.a = 1.0;

        // Calculate the midpoint of the cone using the centroid of inlier points
        pcl::PointXYZ midpoint;
        for (size_t i = 0; i < inliers->indices.size(); ++i) {
            int index = inliers->indices[i];
            midpoint.x += outputCloud.points[index].x;
            midpoint.y += outputCloud.points[index].y;
            midpoint.z += outputCloud.points[index].z;
        }

        // Calculate the centroid
        midpoint.x /= inliers->indices.size();
        midpoint.y /= inliers->indices.size();
        midpoint.z /= inliers->indices.size();

        // Add midpoint to the marker
        geometry_msgs::Point cone_point;
        cone_point.x = midpoint.x;
        cone_point.y = midpoint.y;
        cone_point.z = midpoint.z;
        //cone_marker.points.push_back(cone_point);

        */
 }


int main(int a, char** aa)
{
    ros::init(a, aa, "filter");
    ros::NodeHandle n;
    ros::Subscriber cloud=n.subscribe("cloud",10,detectCones);
    ros::Publisher filtered_cloud_pub=n.advertise<pcl::PointCloud<pcl::PointXYZ>>("filtered_cloud",10);
     /*sensor_msgs::PointCloud2 temp;
        pcl::toROSMsg(outputCloud,temp);
        temp.header.frame_id = "filter";*/

        // publish point cloud
        filtered_cloud_pub.publish(outputCloud);
    

    ros::spin();
}
