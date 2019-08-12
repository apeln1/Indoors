#!/usr/bin/python3
import rospy
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseArray, PoseStamped, Pose
from Grid import Grid
from WPmonitoring import WPmonitoring
import numpy as np
import math

trajectoryOut = PoseArray()
grid = Grid()

flag1 = True
flag2 = True
flag3 = True

drone_pos = None
drone_yaw = None
trajectory = None

def quaternion_to_euler(vec):
    x,y,z,w = vec
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
# callbacks
def occupancyGrid_cb(data):
    global grid, flag1
    # height,width - [m]
    # data.info.width,data.info.height [px]
    # res [m/px]
    grid.res = data.info.resolution
    width = int(grid.res * data.info.width)
    height = int(grid.res * data.info.height)
    grid.matrix = np.reshape(data.data, (data.info.width, data.info.height))
    grid.x_lim = [0, width]
    grid.y_lim = [0, height]
    flag1 = False

    # occupancyGrid = data

def trajectory_cb(data):
    global trajectory, next_yaw, flag3
    print(len(data.poses))
    flag3 = False
    trajectory = []
    for k in data.poses:
        temp = [k.position.x, k.position.y]
        quaternions = np.asarray((k.orientation.x, k.orientation.y, k.orientation.z, k.orientation.w))
        next_yaw = quaternion_to_euler(quaternions)[2]

        trajectory.append(temp)


def odometry_cb(data):
    global drone_pos, drone_yaw, flag2
    drone_pos = [[data.pose.pose.position.x, data.pose.pose.position.y]]
    quaternions = np.asarray((data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z,
                              data.pose.pose.orientation.w))
    drone_yaw = quaternion_to_euler(quaternions)[2]
    flag2 = False
    try:
        drone_pos = np.add(drone_pos, [grid.x_lim[1]/2, grid.y_lim[1]/2])
    except:
        flag2 = True

# subscribers
rospy.Subscriber("/occ_map_yuval", OccupancyGrid, occupancyGrid_cb)
rospy.Subscriber("/route_planner/out/local_trajectory", PoseArray, trajectory_cb)
rospy.Subscriber("/mavros/local_position/odom", Odometry, odometry_cb)


# pubWaypoint = rospy.Publisher('/route_planner/out/next_pose', PoseStamped , queue_size=10)

pubWaypoint = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped , queue_size=10)

pubTraj = rospy.Publisher('/route_planner/out/wp_trajectory', PoseArray,  queue_size=10)
pub = rospy.Publisher('topic_name', String, queue_size=10)
rospy.init_node('wp_monitoring')
r = rospy.Rate(10) # 50hz

while flag1 or flag2 or flag3:
    r.sleep()
    pub.publish("while")

i = 0
counter = 0
wpMonitoring = WPmonitoring(i, grid.x_lim, grid.y_lim, grid.res, drone_pos)

while not rospy.is_shutdown():
    wpMonitoring.FollowWP(drone_pos, drone_yaw, grid.matrix, trajectory, next_yaw)
    for t in wpMonitoring.DroneTrj:
        pose = Pose()
        print([t[0],t[1]])
        pose.position.x = t[0] - grid.x_lim[1]/2
        pose.position.y = t[1] - grid.y_lim[1]/2
        pose.position.z = 4

        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = 0
        pose.orientation.w = 0
        trajectoryOut.poses.append(pose)
    trajectoryOut.header.stamp = rospy.get_rostime()
    trajectoryOut.header.seq = counter

    nextPose = PoseStamped()
    nextPose.pose.position.x = wpMonitoring.next_pos[0][0] - grid.x_lim[1]/2
    nextPose.pose.position.y = wpMonitoring.next_pos[0][1] - grid.y_lim[1]/2
    nextPose.pose.position.z = 4
    nextPose.pose.orientation.x = 0
    nextPose.pose.orientation.y = 0
    nextPose.pose.orientation.z = np.cos(wpMonitoring.next_heading / 2)
    nextPose.pose.orientation.w = np.cos(wpMonitoring.next_heading / 2)
    nextPose.header.frame_id = 'local_origin_ned'

    pubWaypoint.publish(nextPose)
    pubTraj.publish(trajectoryOut)
    trajectoryOut = PoseArray()
    counter+=1
    pub.publish("hello world")
    r.sleep()
