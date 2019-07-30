
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

//    geometry_msgs::PoseStamped pose;
//    pose.pose.position.x = 0;
//    pose.pose.position.y = 0;
//    pose.pose.position.z = 2;

    mavros_msgs::PositionTarget posTarget;

    posTarget.position.x = 0;
    posTarget.position.y = 0;
    posTarget.position.z = 1.2;

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



        if(armed && (ros::Time::now() - last_update>ros::Duration(30.0)) && current_state.mode == "OFFBOARD")
        {
            //posTarget.position.x = 1.0;
            offb_set_mode.request.custom_mode = "AUTO.LAND";
            set_mode_client.call(offb_set_mode);
	    break;
        }
        if(armed && (ros::Time::now() - last_update>ros::Duration(35.0)) && current_state.mode == "OFFBOARD")
        {
	    posTarget.yaw = 3.14/2;
            //offb_set_mode.request.custom_mode = "AUTO.LAND";
            //set_mode_client.call(offb_set_mode);
        }

        if(armed && (ros::Time::now() - last_update>ros::Duration(35.0)) && current_state.mode == "OFFBOARD")
        {

                posTarget.position.y = 1.0;
        }
        if(armed && (ros::Time::now() - last_update>ros::Duration(45.0)) && current_state.mode == "OFFBOARD")
        {
		posTarget.yaw = 3.14;
        }

        if(armed && (ros::Time::now() - last_update>ros::Duration(55.0)) && current_state.mode == "OFFBOARD")
        {
            //ROS_INFO("TIMEOUT EXCEEDED");
            //offb_set_mode.request.custom_mode = "MANUAL";
            //set_mode_client.call(offb_set_mode);
            //arm_cmd.request.value = false;
            //arming_client.call(arm_cmd);
            //break;
	    //

            posTarget.position.x = 0;


            //posTarget.velocity.y = 0.1;

              //break;
        }
        if(armed && (ros::Time::now() - last_update>ros::Duration(65.0)) && current_state.mode == "OFFBOARD")
        {
            //ROS_INFO("TIMEOUT EXCEEDED");
            //offb_set_mode.request.custom_mode = "MANUAL";
            //set_mode_client.call(offb_set_mode);
            //arm_cmd.request.value = false;
            //arming_client.call(arm_cmd);
            //break;
	    //

	    posTarget.yaw = 3.14+3.14/2;

            //posTarget.velocity.y = 0.1;

              //break;
        }

        if(armed && (ros::Time::now() - last_update>ros::Duration(75.0)) && current_state.mode == "OFFBOARD")
	{
              posTarget.position.y = 0;
	}
        if(armed && (ros::Time::now() - last_update>ros::Duration(85.0)) && current_state.mode == "OFFBOARD")

	{
	      posTarget.yaw = 3.14*2;
	}


        pos_Target_pub.publish(posTarget);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
