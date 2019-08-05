#!/usr/bin/python3
import rospy
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from trajectory_msgs.msg import JointTrajectory
from LocalMissionPlanner import LocalMissionPlanner
from EnvSim import Env
from Grid import Grid


occupancyGrid = OccupancyGrid()
odometry = Odometry()
jointTrajectory = JointTrajectory()


# callbacks
def occupancyGrid_cb(data):
    global occupancyGrid
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.header.seq)
    occupancyGrid = data

def odometry_cb(data):
    global odometry
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.header.seq)
    odometry = data

def joint_trajectory_cb(data):
    global jointTrajectory
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.header.seq)
    jointTrajectory = data

# subscribers
rospy.Subscriber("/route_planner/occupancy_grid", OccupancyGrid, occupancyGrid_cb)
rospy.Subscriber("/route_planner/odometry", Odometry, odometry_cb)
rospy.Subscriber("/route_planner/joint_trajectory", JointTrajectory, joint_trajectory_cb)


pub = rospy.Publisher('topic_name', String, queue_size=10)
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
trajectory=[]

local_mission_planner = LocalMissionPlanner(env_limits, i, grid.res, drone_pos, grid.matrix, current_heading)
while not rospy.is_shutdown():
   local_mission_planner.TaskAssignment(dict_of_drones_pos, drone_pos,
                  drone_pos, grid.matrix,
                  yaw, trajectory)


   pub.publish("hello world")
   r.sleep()
