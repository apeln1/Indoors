#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from LocalMissionPlanner import LocalMissionPlanner
from EnvSim import Env
from Grid import Grid

pub = rospy.Publisher('topic_name', String, queue_size=10)
rospy.init_node('node_name')
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