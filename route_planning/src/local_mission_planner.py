#!/usr/bin/python3
import rospy
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from mavros_msgs.msg import BatteryStatus
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose, PoseStamped
from LocalMissionPlanner import LocalMissionPlanner
from EnvSim import Env
from Grid import Grid
import numpy as np
import math
from copy import deepcopy as dcpy


occupancyGrid = OccupancyGrid()
odometry = Odometry()
trajectoryIn = PoseArray()
trajectoryOut = PoseArray()
batteryStatus = BatteryStatus()
grid = Grid()

flag1 = True
flag2 = True
flag3 = True

drone_pos = None
drone_yaw = None
trajectory = None

rospy.init_node('local_mission_planner', anonymous=True)
pub = rospy.Publisher('topic_name', String, queue_size=10)

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
    return [roll, pitch, yaw]

# callbacks
def occupancyGrid_cb(data):
    global grid, flag1
    # height,width - [m]
    # data.info.width,data.info.height [px]
    # res [m/px]
    grid.res = data.info.resolution
    width = int(grid.res * data.info.width)
    height = int(grid.res * data.info.height)
    grid.matrix = np.reshape(data.data, (data.info.height, data.info.width))
    grid.x_lim = [-width/2, width/2]
    grid.y_lim = [-height/2, height/2]
    flag1 = False
    # occupancyGrid = data

def odometry_cb(data):
    global drone_pos, drone_yaw, flag2
    drone_pos = [[data.pose.pose.position.x, data.pose.pose.position.y]]
    quaternions = np.asarray((data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z,
                              data.pose.pose.orientation.w))
    drone_yaw = quaternions[2]
    flag2 = False
    # odometry = data

def trajectory_cb(data):
    global trajectory, flag3
    if data._type == 'geometry_msgs/Pose':
        trajectory = [[data.position.x, data.position.y]]
    else:
        trajectory = []
        for k in data.poses:
            trajectory.append([k.position.x, k.position.y])
    flag3 = False
    # trajectoryIn = data

def battery_status_cb(data):
    global batteryStatus
    batteryStatus = data

# subscribers
rospy.Subscriber("/occ_map_yuval", OccupancyGrid, occupancyGrid_cb)
#   rospy.Subscriber("/route_planner/in/odometry", Odometry, odometry_cb)
rospy.Subscriber("/mavros/local_position/odom", Odometry, odometry_cb)
# rospy.Subscriber("/route_planner/in/nextPose", Pose, trajectory_cb)
rospy.Subscriber("/route_planner/in/trajectory", PoseArray, trajectory_cb)
#   rospy.Subscriber("/mavros/battery", BatteryStatus, battery_status_cb)



pubTrajectory = rospy.Publisher('/route_planner/out/trajectory', PoseArray, queue_size=10)
pubNextPose = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
r = rospy.Rate(10)   # 10hz

# wait for initial topics
while flag1 or flag2 or flag3:
    r.sleep()
    pub.publish("while")

i = 0
# env = Env(resolution, 2)
# dict_of_drones_pos = dict()
# yaw = 3.14
# grid = Grid(env.border_polygon_for_grid, resolution)
env_limits = grid.x_lim + grid.y_lim

counter = 0

local_mission_planner = LocalMissionPlanner(env_limits, i, grid.res, drone_pos, grid.matrix, drone_yaw)
while not rospy.is_shutdown():
    local_mission_planner.TaskAssignment(None, drone_pos,
                  None, grid.matrix,
                  drone_yaw, trajectory)

    for t in local_mission_planner.traj:
        pose = Pose()
        pose.position.x = t[0]
        pose.position.y = t[1]
        #pose.position.z = t[1]  what about z?????????

        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = np.sin(local_mission_planner.curyaw / 2)
        pose.orientation.w = np.cos(local_mission_planner.curyaw / 2)
        trajectoryOut.poses.append(pose)

    trajectoryOut.header.stamp = rospy.get_rostime()
    trajectoryOut.header.seq = counter

    counter+=1

    pub.publish("hello world")
    pubTrajectory.publish(trajectoryOut)
    nextPose = PoseStamped()
    nextPose.pose = dcpy(trajectoryOut.poses[0])
    nextPose.header.frame_id = 'local_origin_ned'
    nextPose.pose.position.z = 4.0
    trajectoryOut = PoseArray()
    # nextPoseStamped = pos
    # print(nextPose)
    pubNextPose.publish(nextPose)
    r.sleep()
