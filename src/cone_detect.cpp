#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include "std_msgs/Float64MultiArray.h"


ros::Publisher cone_center_pub;

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
   
    // Convert sensor_msgs::PointCloud2 to PCL PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // Perform Voxel Grid Downsampling to reduce computation
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.01, 0.01, 0.01);
    sor.filter(*cloud);

    // Build a KdTree for Euclidean Cluster Extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    // Apply Euclidean Cluster Extraction
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.3);  
    ec.setMinClusterSize(5);     
    ec.setMaxClusterSize(1000);   
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    pcl::PointCloud<pcl::PointXYZ> center_points;
    // Process the clusters
    for (const auto& indices : cluster_indices) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
        float x = 0;
        float y = 0;
        for (int index : indices.indices) {
            cluster->push_back((*cloud)[index]);
            
            x = x + (*cloud)[index].x;
            y = y + (*cloud)[index].y;
        }
        pcl::PointXYZ newPoint;
        x = x/indices.indices.size();
        y = y/indices.indices.size();
        newPoint.x = x;
        newPoint.y = y;
        center_points.push_back(newPoint);
    }

    sensor_msgs::PointCloud2 temp;
    pcl::toROSMsg(center_points,temp);

    cone_center_pub.publish(temp);
    
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "cluster_extraction");
    ros::NodeHandle nh("~");

    ros::Subscriber sub = nh.subscribe("/cloud", 1, cloudCallback);
    cone_center_pub = nh.advertise<sensor_msgs::PointCloud2>("/cone_center",1);

    ros::spin();

    return 0;
}
