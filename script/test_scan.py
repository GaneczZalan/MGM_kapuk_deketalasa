#!/usr/bin/env python3

import math

import rospy

from sensor_msgs.msg import PointCloud2, LaserScan
import sensor_msgs.point_cloud2 as pcl2


import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud


class tf_pub:
    def __init__(self):

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.cloud_pub = rospy.Publisher("/cloud",PointCloud2, queue_size=1)
        self.sub = rospy.Subscriber("/agent1/scan", LaserScan, self.scan_callback, queue_size=1)



    def scan_callback(self, scan_in: LaserScan):


        newPoint = []
        for i in range(0,len(scan_in.ranges),5):
            angle = scan_in.angle_min + i * scan_in.angle_increment
            x = scan_in.ranges[i] * math.cos(angle)
            y = scan_in.ranges[i] * math.sin(angle)
            z = 0
            newPoint.append([x,y,z])

        localCloud = pcl2.create_cloud_xyz32(scan_in.header, newPoint)

        if self.tfBuffer.can_transform("map", scan_in.header.frame_id, scan_in.header.stamp, rospy.Duration(0.1)):
            # Getting the transformation
            trans_base2map = self.tfBuffer.lookup_transform("map",scan_in.header.frame_id, scan_in.header.stamp)

            mapCloud = do_transform_cloud(localCloud, trans_base2map)
            #mapCloud.header.frame_id = "map"

            self.cloud_pub.publish(mapCloud)
        


if __name__ == '__main__':
    rospy.init_node("test_tf")

    tf_publisher = tf_pub()
    
    rospy.spin()
