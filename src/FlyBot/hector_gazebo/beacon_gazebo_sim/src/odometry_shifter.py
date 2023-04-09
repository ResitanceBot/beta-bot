#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import numpy as np

rospy.init_node("odometry_shifter")
rospy.sleep(0.1)

pub_odom = rospy.Publisher("odom", Odometry)


def odom_clb(msg: Odometry):
    global pub_odom
    msg_out = Odometry()
    msg_out = msg
    msg_out.header.frame_id = "odom"
    msg_out.pose.pose.position.x = msg.pose.pose.position.x * np.random.normal(1.4, 0.0001, 1)[0]
    msg_out.pose.pose.position.y = msg.pose.pose.position.y * np.random.normal(1.4, 0.0001, 1)[0]
    msg_out.pose.pose.position.z = msg.pose.pose.position.z * np.random.normal(1.4, 0.0001, 1)[0]
    pub_odom.publish(msg_out)


sub_odom = rospy.Subscriber(f"odom_sim", Odometry, odom_clb)
rospy.loginfo("odometry_shifter: Hello ")
rospy.spin()