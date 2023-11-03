#!/usr/bin/env python3

import rospy

from std_msgs.msg import String

def callback_string(msg):
    rospy.logwarn("I heard %s", msg.data)


if __name__ == '__main__':
    rospy.init_node("test_node_1")
    sub = rospy.Subscriber("/topic1", String, callback_string, queue_size=1000)
    pub = rospy.Publisher("/topic1",String, queue_size=1000)

    r = rospy.Rate(1)

    while not rospy.is_shutdown():
        msg = String()
        msg.data = "hello"
        pub.publish(msg)
        rospy.loginfo("Sending..")

        r.sleep()