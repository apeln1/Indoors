#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/QuaternionStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PointStamped.h"

#include <string>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/StreamRate.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include "tf/LinearMath/Transform.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <ros/console.h>

static mavros_msgs::State current_state;
static sensor_msgs::Imu current_imu;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void mav_imu_data_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
    current_imu = *msg;
}

int main(int argc, char **argv)
{
    uint count = 0;
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);

    ros::Publisher mav_thr_pub = nh.advertise<std_msgs::Float64>( "mavros/setpoint_attitude/att_throttle", 100);
    ros::Publisher mav_att_pub = nh.advertise<geometry_msgs::PoseStamped>( "mavros/setpoint_attitude/attitude", 100 );

    ros::ServiceClient stream_rate_client = nh.serviceClient<mavros_msgs::StreamRate>( "mavros/set_stream_rate");

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    ros::Subscriber mav_imu_data_sub = nh.subscribe<sensor_msgs::Imu>( "mavros/imu/data", 100, mav_imu_data_callback );

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0); //change to 50??

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }


    //throttle
    std_msgs::Float64 cmd_thr;

    // attitude
    geometry_msgs::PoseStamped cmd_att;

    // throttle value
    cmd_thr.data = 0.5;


    ////////////////////// Set stream rate/////////////////////////////////
    mavros_msgs::StreamRate streamRate;

    streamRate.request.stream_id = 0;
    streamRate.request.message_rate = 100;
    streamRate.request.on_off = 1;

    if (true == stream_rate_client.call(streamRate))
    {
        ROS_INFO("OK. Stream rate set");
    }
    else
    {
        ROS_INFO("Failed to call service: stream rate");
    }

    ////////////////////// /Set stream rate/////////////////////////////////

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "GUIDED_NOGPS";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "GUIDED_NOGPS" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("GUIDED enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }


        mav_thr_pub.publish( cmd_thr );


        // Create attitude command message
        cmd_att.header.stamp = ros::Time::now();
        cmd_att.header.seq = count;
        cmd_att.pose.position.x = 0.0;
        cmd_att.pose.position.y = 0.0;
        cmd_att.pose.position.z = 0.0;

        double roll = 0;
        double pitch = 0;
        double yaw = 0;
        // desired angles
        tf::Quaternion mav_orientation = tf::createQuaternionFromRPY( roll, pitch, yaw );

        cmd_att.pose.orientation.x = mav_orientation.x();
        cmd_att.pose.orientation.y = mav_orientation.y();
        cmd_att.pose.orientation.z = mav_orientation.z();
        cmd_att.pose.orientation.w = mav_orientation.w();

        mav_att_pub.publish( cmd_att );


        ++count;
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
