
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/TwistStamped.h>


static mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
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

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    mavros_msgs::PositionTarget posTarget;

    posTarget.position.x = 0;
    posTarget.position.y = 0;
    posTarget.position.z = 10;

//    posTarget.velocity.x = 1;
    posTarget.velocity.z = 1;
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

    while(ros::ok()){
        if(current_state.mode!="OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(1.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("OFFBOARD enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(1.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        if(ros::Time::now() - last_update>ros::Duration(15.0))
        {
            //posTarget.velocity.x = 0.1;
            //posTarget.velocity.y = 0.1;

	    ROS_INFO("Timeout exceeded");	    
            //offb_set_mode.request.custom_mode = "MANUAL";
	    if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
	    {
		    ROS_INFO("MANUAL set");
	    }
            arm_cmd.request.value = false;
	    if(arming_client.call(arm_cmd) && arm_cmd.response.success)
	    {
		break;
	    }
            
        }

        if(ros::Time::now() - last_update>ros::Duration(2.0))
        {


              //break;
        }

//        local_pos_pub.publish(pose);
//        local_pos_vel.publish(twist);
        //pos_Target_pub.publish(posTarget);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
