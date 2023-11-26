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



struct scan2pcl{

    sensor_msgs::PointCloud2 mapCloud;

	tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

    ros::Subscriber scan_sub;
    ros::Publisher cloud_pub;
    ros::NodeHandle nh;
    ros::Publisher marker_pub;
    ros::Publisher filtered_cloud_pub;
    
    scan2pcl(ros::NodeHandle nh_):nh(nh_),tfListener(tfBuffer){
        cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud", 1);
        scan_sub = nh.subscribe("scan", 1, &scan2pcl::scan_callback, this);
        marker_pub = nh.advertise<visualization_msgs::MarkerArray>("cone_markers", 1);
        

    }

    // callback for laser scan
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
    {
        // Get the points from the scan message
        pcl::PointCloud<pcl::PointXYZ> localCloud;
        for (int i=0; i<scan_in->ranges.size(); i ++)
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
        cloud_pub.publish(temp);

    }

     
};



int main(int a, char** aa) {
	ros::init(a, aa, "map");
	ros::NodeHandle n;

    scan2pcl scan_converter(n);

    ros::spin();

}