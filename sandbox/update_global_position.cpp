#include <ros/ros.h>
#include <iostream>
#include <ros/time.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/TimesyncStatus.h>
#include <sensor_msgs/Imu.h>

static geometry_msgs::PoseStamped msg_vicon_pose;
static geometry_msgs::PoseStamped msg_vicon_pose_launch;
static ros::Time visionTime;
static uint64_t delta;
static bool isOdom = false;

void Callback_fcu_pose(const geometry_msgs::PoseStamped msg_fcu_pose)
{
    if(msg_fcu_pose.pose.position.z>=0.5)
    {
      isOdom = true;
    }

    ROS_INFO("Got data : %f, %f, %f", msg_fcu_pose.pose.position.x, msg_fcu_pose.pose.position.y, msg_fcu_pose.pose.position.z);
}

void callback_timesync_status(const mavros_msgs::TimesyncStatus timesync_status)
{
    //ROS_INFO("Got data : %llu", timesync_status.observed_offset_ns);
    //delta = timesync_status.observed_offset_ns;
}


void callback_imu_from_structure(const sensor_msgs::Imu imuData)
{
//    ROS_INFO("Got data : %f", imuData.angular_velocity.x);

//    msg_vicon_pose.header.stamp       = imuData.header.stamp;
//    msg_vicon_pose.pose.position.x    = imuData.linear_acceleration.x;
//    msg_vicon_pose.pose.position.y    = imuData.linear_acceleration.y;
//    msg_vicon_pose.pose.position.z    = imuData.linear_acceleration.z;

//    msg_vicon_pose.pose.orientation.x = imuData.angular_velocity.x;
//    msg_vicon_pose.pose.orientation.y = imuData.angular_velocity.y;
//    msg_vicon_pose.pose.orientation.z = imuData.angular_velocity.z;
}

void odom_cb(const nav_msgs::Odometry odometry)
{
    msg_vicon_pose.header.stamp       = odometry.header.stamp;
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

    ros::Subscriber timeSyncStatusSub = n.subscribe("/mavros/timesync_status",10,callback_timesync_status);

    ros::Subscriber imuDataFromStructureSub = n.subscribe("/imu_node/imu",10,callback_imu_from_structure);

    ros::Subscriber vicon_sub = n.subscribe("/mavros/local_position/pose",100, Callback_fcu_pose);

    ros::Rate loop_rate(30);


    while (ros::ok())
    {
        loop_rate.sleep();
//        ros::Time t;
//        uint64_t fixed;
//        fixed = delta - lastTime;

        //msg_vicon_pose.header.stamp = visionTime;
        //ROS_INFO("Time fix : %llu", msg_vicon_pose.header.stamp.sec);
        //ROS_INFO("Time fix now: %llu", ros::Time::now().sec);

        //msg_vicon_pose.header.frame_id = "fcu"; //optional. Works fine without frame_id
        //msg_vicon_pose.pose.position.x-=0.002;
//        msg_vicon_pose.pose.position.x = 0;
//        msg_vicon_pose.pose.position.y = 0;
//        msg_vicon_pose.pose.position.z = 0;

//        msg_vicon_pose.pose.orientation.w = 1;
//        msg_vicon_pose.pose.orientation.x = 0;
//        msg_vicon_pose.pose.orientation.y = 0;
//        msg_vicon_pose.pose.orientation.z = 0;

        if(isOdom)
        {
          vicon_pub.publish(msg_vicon_pose);
        }
        else{
          msg_vicon_pose_launch.header.stamp = ros::Time::now();

          msg_vicon_pose_launch.pose.position.x = 0;
          msg_vicon_pose_launch.pose.position.y = 0;


          msg_vicon_pose_launch.pose.orientation.w = 1;
          msg_vicon_pose_launch.pose.orientation.x = 0;
          msg_vicon_pose_launch.pose.orientation.y = 0;
          msg_vicon_pose_launch.pose.orientation.z = 0;
          vicon_pub.publish(msg_vicon_pose_launch);

        }


        ros::spinOnce();
    }
    return 0;
}
