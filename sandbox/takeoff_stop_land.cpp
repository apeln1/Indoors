
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/TwistStamped.h>


static geometry_msgs::PoseStamped msg_vicon_pose;
static mavros_msgs::State current_state;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}


void Callback_fcu_pose(const geometry_msgs::PoseStamped msg_fcu_pose_dummy)
{
    msg_vicon_pose=msg_fcu_pose_dummy;
    //ROS_INFO("Got data : %f, %f, %f", msg_fcu_pose_dummy.pose.position.x, msg_fcu_pose_dummy.pose.position.y, msg_fcu_pose_dummy.pose.position.z);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);

    ros::Subscriber vicon_sub = nh.subscribe("/mavros/local_position/pose",100, Callback_fcu_pose);

//    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
//            ("mavros/setpoint_position/local", 10);
    ros::Publisher pos_Target_pub = nh.advertise<mavros_msgs::PositionTarget>
            ("/mavros/setpoint_raw/local", 10);
//    ros::Publisher local_pos_vel = nh.advertise<geometry_msgs::TwistStamped>
//            ("/mavros/setpoint_velocity/cmd_vel", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

//    geometry_msgs::PoseStamped pose;
//    pose.pose.position.x = 0;
//    pose.pose.position.y = 0;
//    pose.pose.position.z = 2;

    mavros_msgs::PositionTarget posTarget;

    posTarget.position.x = 0;
    posTarget.position.y = 0;
    posTarget.position.z = 1.0;

//    posTarget.velocity.x = 1;
//    posTarget.velocity.z = 1;
    //posTarget.yaw_rate = 50;
//    posTarget.

//    geometry_msgs::TwistStamped twist;
//    twist.twist.linear.x = 0;
//    twist.twist.linear.y = 0;
//    twist.twist.linear.z = 1;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        pos_Target_pub.publish(posTarget);
        //local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    ros::Time last_update = ros::Time::now();
    bool armed = false;
    bool offBoardSet = false;
    int countToLanding = 0;
    bool needsToLand = false;

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" && !offBoardSet &&
            (ros::Time::now() - last_request > ros::Duration(1.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("OFFBOARD enabled");
                offBoardSet = true;
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(1.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                    last_update = ros::Time::now();
                    armed = true;
                }
                last_request = ros::Time::now();
            }
        }



        if(armed && (ros::Time::now() - last_update>ros::Duration(7.0)) && current_state.mode == "OFFBOARD")
        {
            posTarget.position.x = 4;

        }
  if(current_state.mode == "OFFBOARD" && msg_vicon_pose.pose.position.x>1.0 /*&& !needsToLand*/)
  {
    posTarget.position.x = msg_vicon_pose.pose.position.x;
    ROS_INFO("STOPPING!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    needsToLand = true;
  }

  if(current_state.mode == "OFFBOARD" && needsToLand)
  {
     countToLanding++;
     if(countToLanding>80)
     {
                offb_set_mode.request.custom_mode = "AUTO.LAND";
              set_mode_client.call(offb_set_mode);
    break;
     }

  }






        pos_Target_pub.publish(posTarget);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
