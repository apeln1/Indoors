#!/usr/bin/env python

import roslib
import rospy
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from copy import deepcopy as dcpy

traj_pub = rospy.Publisher("/route_planner/in/trajectory", PoseArray, queue_size=10)
pose_pub = rospy.Publisher("/route_planner/in/nextPose", Pose, queue_size=10)
arr_msg = PoseArray()
pose_msg = Pose()


def callback_traj(msg):
    global arr_msg, pose_msg, traj_pub, pose_pub
    msg.pose.position.z = 4
    pose_msg = dcpy(msg.pose)
    # pose_msg.orientation = msg.pose.orientation
    # pose_msg.position = msg.pose.position
    arr_msg.poses.insert(callback_traj.counter, pose_msg)
    arr_msg.header.frame_id = msg.header.frame_id
    callback_traj.counter += 1
    # if you want to publish the arr_msg at 10 Hz then cut the line below to the while loop in talker
    # now its buplished every time a point is addad
    # traj_pub.publish(arr_msg)

callback_traj.counter = 0

def talker():
    global arr_msg
    rospy.init_node('Traj_yuval', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, callback_traj)
    while not rospy.is_shutdown():
        traj_pub.publish(arr_msg)
        pose_pub.publish(pose_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass


