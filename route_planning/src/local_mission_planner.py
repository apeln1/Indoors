#!/usr/bin/python3
import rospy
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from mavros_msgs.msg import BatteryStatus
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from LocalMissionPlanner import LocalMissionPlanner
from EnvSim import Env
from Grid import Grid
import numpy as np


occupancyGrid = OccupancyGrid()
odometry = Odometry()
trajectory = PoseArray()
trajectoryOut = PoseArray()
batteryStatus = BatteryStatus()



# callbacks
def occupancyGrid_cb(data):
    global occupancyGrid
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.header.seq)
    occupancyGrid = data

def odometry_cb(data):
    global odometry
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.header.seq)
    odometry = data

def trajectory_cb(data):
    global jointTrajectory
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.header.seq)
    jointTrajectory = data

def battery_status_cb(data):
    global batteryStatus
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.header.seq)
    batteryStatus = data

# subscribers
rospy.Subscriber("/route_planner/in/occupancy_grid", OccupancyGrid, occupancyGrid_cb)
rospy.Subscriber("/route_planner/in/odometry", Odometry, odometry_cb)
rospy.Subscriber("/route_planner/in/trajectory", PoseArray, trajectory_cb)
rospy.Subscriber("/mavros/battery", BatteryStatus, battery_status_cb)


pub = rospy.Publisher('topic_name', String, queue_size=10)
pubTrajectory = rospy.Publisher('/route_planner/out/trajectory', PoseArray, queue_size=10)
rospy.init_node('local_mission_planner')
r = rospy.Rate(10) # 10hz


resolution = 10
i = 0
drone_pos = [[-10000, -10000]]
current_heading = 3.14

env = Env(resolution, 2)
dict_of_drones_pos = dict()
yaw = 3.14
grid = Grid(env.border_polygon_for_grid, resolution)
env_limits = grid.x_lim + grid.y_lim
trajectory = list()
counter = 0
local_mission_planner = LocalMissionPlanner(env_limits, i, grid.res, drone_pos, grid.matrix, current_heading)
while not rospy.is_shutdown():
    local_mission_planner.TaskAssignment(dict_of_drones_pos, drone_pos,
                  drone_pos, grid.matrix,
                  yaw, trajectory)

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
    r.sleep()
