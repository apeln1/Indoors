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

# initialize node
rospy.init_node('occupancy_grid_node')
point_msg = []

# initialize map, in the future get one from slam
map_msg = OccupancyGrid()
map_msg.header.frame_id = 'local_origin_ned'
rate = 1.0
resolution = 0.1  # [m/px]
width = 50  # [m]
height = 50  # [m]
width_px = int(width / resolution)
height_px = int(height / resolution)

def set_obstacle(grid, point):
    # set the occupied cells when detecting an obstacle
    # grid:				ndarray [width,height]
    # point             ndarray [x,y,z]
    global resolution
    obstacle = np.array(point[:-1])
    # from meter to map resulotion
    obstacle /= resolution
    # transform to matrix
    obstacle += [width_px // 2, height_px // 2]
    # set probability of occupancy to 100 and neighbour cells to 50
    grid[int(obstacle[0]), int(obstacle[1])] = int(100)


def quaternion_to_euler(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return [yaw, pitch, roll]

def callback_range(msg):
    # callback range
    global point_msg
    point_msg = msg

# Subscribers
#/local_pointcloud #/stereo/points2
point_scan = rospy.Subscriber("/local_pointcloud", PointCloud2, callback_range)

occ_pub = rospy.Publisher("/occ_map_yuval", OccupancyGrid, queue_size = 10)

# main function
if __name__ == '__main__':

    # fill map_msg with the parameters from launchfile
    map_msg.info.resolution = resolution
    map_msg.info.width = width_px
    map_msg.info.height = height_px
    map_msg.data = range(width_px * height_px)

    map_msg.info.origin.position.x = - width // 2
    map_msg.info.origin.position.y = - height // 2

    grid = numpy.ndarray((width_px, height_px), buffer=numpy.zeros((width_px, height_px), dtype=numpy.int),
                     dtype=numpy.int)
    grid.fill(int(0))

    loop_rate = rospy.Rate(rate)

    while not rospy.is_shutdown():
        grid
        # grid[grid != -1] -= 20
        # grid[grid < 10] = 0
        try:
            for point in sensor_msgs.point_cloud2.read_points(point_msg, skip_nans=True):
                set_obstacle(grid, point[:-1])
        except:
            continue
        # for i in range(width_px * height_px):
        #     map_msg.data[i] = grid.flat[i]
        map_msg.data = grid.flatten()

        occ_pub.publish(map_msg)

        loop_rate.sleep()