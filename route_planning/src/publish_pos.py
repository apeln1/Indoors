#!/usr/bin/env python

import roslib
import rospy
import numpy
import math
import numpy as np
import rosbag
import sensor_msgs.point_cloud2
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
# import tf
from geometry_msgs.msg import PoseStamped, PointStamped, Point, Pose
from mavros_msgs.msg import PositionTarget
from mavros_msgs.msg import Trajectory
import time

# rospy.init_node('Trajectory_yuval')
pose_msg = PoseStamped()
pose_msg.header.frame_id = "/local_origin"

pose_msg.pose.position.x = -2.0
pose_msg.pose.position.y = 1.0
pose_msg.pose.position.z = 4.0
pose_msg.pose.orientation.x = 0.0
pose_msg.pose.orientation.y = 0.0
pose_msg.pose.orientation.z = 0.770876765251
pose_msg.pose.orientation.w = 0.636984288692

first_time = time.time()

def talker():
    pose_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size = 10)
    rospy.init_node('Pose_yuval', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        # hello_str = "hello world %s" % rospy.get_time()
        # rospy.loginfo(hello_str)
        pose_msg.pose.position.y = abs(float((time.time() - first_time) % 20 - 10))
        ros_time = rospy.rostime.get_rostime()
        # pose_msg.header.stamp.nsecs = ros_time.nsecs
        # pose_msg.header.stamp.secs = ros_time.secs
        pose_pub.publish(pose_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass


