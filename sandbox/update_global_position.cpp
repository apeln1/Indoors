#include <ros/ros.h>
#include <iostream>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <nav_msgs/Odometry.h>

static geometry_msgs::PoseStamped msg_vicon_pose;

void Callback_fcu_pose(const geometry_msgs::PoseStamped msg_fcu_pose_dummy)
{
    ROS_INFO("Got data : %f, %f, %f", msg_fcu_pose_dummy.pose.position.x, msg_fcu_pose_dummy.pose.position.y, msg_fcu_pose_dummy.pose.position.z);
}

void odom_cb(const nav_msgs::Odometry odometry)
{
    msg_vicon_pose.pose.position.x    = odometry.pose.pose.position.x;
    msg_vicon_pose.pose.position.y    = odometry.pose.pose.position.y;
    msg_vicon_pose.pose.position.z    = odometry.pose.pose.position.z;

    msg_vicon_pose.pose.orientation.w = odometry.pose.pose.orientation.w;
    msg_vicon_pose.pose.orientation.x = odometry.pose.pose.orientation.x;
    msg_vicon_pose.pose.orientation.y = odometry.pose.pose.orientation.y;
    msg_vicon_pose.pose.orientation.z = odometry.pose.pose.orientation.z;
    //ROS_INFO("Got data : %f, %f, %f", odometry.pose.pose.position.x,odometry.pose.pose.position.y,odometry.pose.pose.position.z);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pub_sub");
    ros::NodeHandle n;

    ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>
            ("odom", 100, odom_cb);
    ros::Publisher vicon_pub = n.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose",100);
    ros::Subscriber vicon_sub = n.subscribe("/mavros/local_position/pose",100, Callback_fcu_pose);

    ros::Rate loop_rate(30);
    int x = 1;

    while (ros::ok())
    {
        loop_rate.sleep();
        msg_vicon_pose.header.stamp = ros::Time::now();
        msg_vicon_pose.header.frame_id = "fcu"; //optional. Works fine without frame_id
        //msg_vicon_pose.pose.position.x-=0.002;
//        msg_vicon_pose.pose.position.x = 0;
//        msg_vicon_pose.pose.position.y = 0;
//        msg_vicon_pose.pose.position.z = 0;

//        msg_vicon_pose.pose.orientation.w = 1;
//        msg_vicon_pose.pose.orientation.x = 0;
//        msg_vicon_pose.pose.orientation.y = 0;
//        msg_vicon_pose.pose.orientation.z = 0;


        vicon_pub.publish(msg_vicon_pose);

        ros::spinOnce();
    }
    return 0;
}
