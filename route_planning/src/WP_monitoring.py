#!/usr/bin/python3
import rospy
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from WPmonitoring import WPmonitoring
from EnvSim import Env
from Grid import Grid


occupancyGrid = OccupancyGrid()
odometry = Odometry()
trajectory = PoseArray()





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



# subscribers
rospy.Subscriber("/route_planner/in/occupancy_grid", OccupancyGrid, occupancyGrid_cb)
rospy.Subscriber("/route_planner/in/odometry", Odometry, odometry_cb)
rospy.Subscriber("/route_planner/out/trajectory", PoseArray, trajectory_cb)



pub = rospy.Publisher('topic_name', String, queue_size=10)
rospy.init_node('wp_monitoring')
r = rospy.Rate(50) # 10hz


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
wpMonitoring = WPmonitoring(i, grid.x_lim, grid.y_lim, grid.res, drone_pos)

while not rospy.is_shutdown():
    wpMonitoring.FollowWP(drone_pos, yaw, grid.matrix, trajectory,
             yaw)

    # for t in local_mission_planner.traj:
    #     pose = Pose()
    #     pose.position.x = t[0]
    #     pose.position.y = t[1]
    #     #pose.position.z = t[1]  what about z?????????
    #
    #     pose.orientation.x = 0
    #     pose.orientation.y = 0
    #     pose.orientation.z = np.sin(local_mission_planner.curyaw / 2)
    #     pose.orientation.w = np.cos(local_mission_planner.curyaw / 2)
    #     trajectoryOut.poses.append(pose)
    #
    # trajectoryOut.header.stamp = rospy.get_rostime()
    # trajectoryOut.header.seq = counter
    #
    # counter+=1

    pub.publish("hello world")
    r.sleep()
