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

# rospy.init_node('Trajectory_yuval')
traj_msg = Trajectory()
traj_msg.header.frame_id = "/local_origin"
traj_msg.point_1 = PositionTarget()
traj_msg.point_1.position.x = -1.0
traj_msg.point_1.position.y = 8.0
traj_msg.point_1.position.z = 4.0
traj_msg.point_1.yaw = 1.5
traj_msg.point_valid = b'\1\0\0\0\0'


#point_scan = rospy.Subscriber("/local_pointcloud", PointCloud2, callback_range)


# traj_pub.publish(traj_msg)

# rospy.spin()

def talker():
    traj_pub = rospy.Publisher("/mavros/trajectory/generated", Trajectory, queue_size = 10)
    rospy.init_node('Trajectory_yuval', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        # hello_str = "hello world %s" % rospy.get_time()
        # rospy.loginfo(hello_str)
        ros_time = rospy.rostime.get_rostime()
        traj_msg.header.stamp.nsecs = ros_time.nsecs
        traj_msg.header.stamp.secs = ros_time.secs
        traj_pub.publish(traj_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass


