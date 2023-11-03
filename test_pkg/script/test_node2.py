#!/usr/bin/env python3

import rospy

from std_msgs.msg import String

def callback_string(msg):
    rospy.logwarn("I heard %s", msg.data)


if __name__ == '__main__':
    rospy.init_node("test_node_2")
    sub = rospy.Subscriber("/topic1", String, callback_string, queue_size=1000)

    rospy.spin()