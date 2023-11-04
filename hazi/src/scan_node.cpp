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


struct scan2pcl{

    // sensor_msgs::PointCloud2 mapCloud;


	tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

    ros::Subscriber scan_sub;
    ros::Publisher cloud_pub;
    ros::NodeHandle nh;
    
    
    scan2pcl(ros::NodeHandle nh_):nh(nh_),tfListener(tfBuffer){
        cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud", 1);
        scan_sub = nh.subscribe("/agent1/scan", 1, &scan2pcl::scan_callback, this);

    }

    // callback for laser scan
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
    {
        // Get the points from the scan message
        pcl::PointCloud<pcl::PointXYZ> localCloud;
        for (int i=0; i<scan_in->ranges.size(); i += 5)
        {
            float angle = scan_in->angle_min + i*scan_in->angle_increment;

            pcl::PointXYZ newPoint;
            newPoint.x = scan_in->ranges[i] * cos(angle);
            newPoint.y = scan_in->ranges[i] * sin(angle);
            newPoint.z = 0;
            localCloud.points.push_back(newPoint);
        }


        
        pcl::PointCloud<pcl::PointXYZ> mapCloud;
        // Get the transformation from the laser to the map
        if (tfBuffer.canTransform("map",
							scan_in->header.frame_id,
							scan_in->header.stamp,
							ros::Duration(0.1)))
		{
			// Getting the transformation
			geometry_msgs::TransformStamped trans_base2map = tfBuffer.lookupTransform("map",
														scan_in->header.frame_id,
							                            scan_in->header.stamp);
            

            // Transform the points to the map frame
            pcl_ros::transformPointCloud(localCloud, mapCloud, trans_base2map.transform);
        }

        sensor_msgs::PointCloud2 temp;
        pcl::toROSMsg(mapCloud,temp);
        temp.header.frame_id = "map";
        
        // publish point cloud
        cloud_pub.publish(mapCloud);
    }

    void detectCones(const pcl::PointCloud<pcl::PointXYZ>& cloud)
    {
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_CIRCLE3D);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.2);  // Adjust this threshold based on your LiDAR data and cone size.

        seg.setInputCloud(cloud.makeShared());
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.size() == 0)
        {
            ROS_INFO("No cone detected.");
            return;
        }

        pcl::PointCloud<pcl::PointXYZ> coneCloud;
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud.makeShared());
        extract.setIndices(inliers);
        extract.filter(coneCloud);
        
        // Now 'coneCloud' contains the points representing the detected cones.
        // You can further process and analyze these points to identify cones.

        // For more accurate cone detection, you may want to use more sophisticated algorithms
        // and consider factors like cone size, orientation, and filtering noise.
    }

    
    
};

int main(int a, char** aa) {
	ros::init(a, aa, "scan2pcl");
	ros::NodeHandle n("~");

    scan2pcl scan_converter(n);

    ros::spin();

}