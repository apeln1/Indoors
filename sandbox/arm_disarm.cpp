#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>



int main(int argc, char **argv)
{
  ros::init(argc, argv, "mavros_takeoff");
  ros::NodeHandle n;

    ////////////////////////////////////////////
    /////////////////GUIDED/////////////////////
    ////////////////////////////////////////////
    ros::ServiceClient cl = n.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    mavros_msgs::SetMode srv_setMode;
    srv_setMode.request.base_mode = 0;
    srv_setMode.request.custom_mode = "OFFBOARD";//"GUIDED";
    if(cl.call(srv_setMode)){
        ROS_ERROR("setmode send ok %d value:", 1);
    }else{
        ROS_ERROR("Failed SetMode");
    }

    ////////////////////////////////////////////
    ///////////////////ARM//////////////////////
    ////////////////////////////////////////////
    ros::ServiceClient arming_cl = n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    mavros_msgs::CommandBool srv;
    srv.request.value = true;
    if(arming_cl.call(srv)){
        ROS_ERROR("ARM send ok %d", srv.response.success);
    }else{
        ROS_ERROR("Failed arming or disarming");
    }

    ////////////////////////////////////////////
    /////////////////TAKEOFF////////////////////
    ////////////////////////////////////////////
    ros::ServiceClient takeoff_cl = n.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
    mavros_msgs::CommandTOL srv_takeoff;
    srv_takeoff.request.altitude = 10;
    srv_takeoff.request.latitude = 0;
    srv_takeoff.request.longitude = 0;
    srv_takeoff.request.min_pitch = 0;
    srv_takeoff.request.yaw = 0;
    if(takeoff_cl.call(srv_takeoff)){
        ROS_ERROR("srv_takeoff send ok %d",1);
    }else{
        ROS_ERROR("Failed Takeoff");
    }

    ////////////////////////////////////////////
    /////////////////DO STUFF///////////////////
    ////////////////////////////////////////////

    ros::Duration(10).sleep();


    ////////////////////////////////////////////
    ///////////////////LAND/////////////////////
    ////////////////////////////////////////////
    ros::ServiceClient land_cl = n.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
    mavros_msgs::CommandTOL srv_land;
    srv_land.request.altitude = 10;
    srv_land.request.latitude = 0;
    srv_land.request.longitude = 0;
    srv_land.request.min_pitch = 0;
    srv_land.request.yaw = 0;
    if(land_cl.call(srv_land)){
        ROS_INFO("srv_land send ok %d", 1);
    }else{
        ROS_ERROR("Failed Land");
    }
}
