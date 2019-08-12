
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/TwistStamped.h>

#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <queue>

class waypoint
{
private:
  double m_time;
  double m_x;
  double m_y;
  double m_z;
  double m_yaw;

public:
  double Time()
  {
    return m_time;
  }
  double X()
  {
    return m_x;
  }
  double Y()
  {
    return m_y;
  }
  double Z()
  {
    return m_z;
  }
  double Yaw()
  {
    return m_yaw;
  }
  waypoint(double time, double x, double y, double z, double yaw):m_time(time),m_x(x),m_y(y),m_z(z),m_yaw(yaw) {}
};

void readFile(const std::string& file_name, std::vector<std::vector<double>>& table)
{
  std::string::size_type sz;
  std::ifstream infile(file_name);
  if (infile.is_open()) {
    std::string tmp;
    int i=1;
    std::vector<double> vec;
    while (!infile.eof()) {
       getline(infile, tmp, ',');
       vec.push_back(std::stod(tmp,&sz));
       if(i%5==0)
       {
         table.push_back(vec);
         vec.clear();
       }
       i++;
       tmp.clear();
    }
  }
  infile.close();
}

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


    mavros_msgs::PositionTarget posTarget;

    posTarget.position.x = 0;
    posTarget.position.y = 0;
    posTarget.position.z = 0;

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


    bool armed = false;
    bool offBoardSet = false;
    bool needsToLand = false;
    int countToLanding = 0;

    std::vector<std::vector<double>> table;
    readFile("/home/makeruser/catkin_ws/src/sandbox/route.txt",table);
    std::queue<waypoint> tasks;
    for(std::vector<double> v :table)
    {
      tasks.push(waypoint(v[0],v[1],v[2],v[3],v[4]));
    }



    ros::Time last_request = ros::Time::now();
    ros::Time start_scenario = ros::Time::now();

    while(ros::ok()){


//        if( current_state.mode != "OFFBOARD" && !offBoardSet &&
//            (ros::Time::now() - last_request > ros::Duration(1.0))){
//            if( set_mode_client.call(offb_set_mode) &&
//                offb_set_mode.response.mode_sent){
//                ROS_INFO("OFFBOARD enabled");
//                offBoardSet = true;
//            }
//            last_request = ros::Time::now();
//        } else {
//            if( !current_state.armed &&
//                (ros::Time::now() - last_request > ros::Duration(1.0))){
//                if( arming_client.call(arm_cmd) &&
//                    arm_cmd.response.success){
//                    ROS_INFO("Vehicle armed");
//                    start_scenario = ros::Time::now();
//                    armed = true;
//                }
//                last_request = ros::Time::now();
//            }
//        }

        armed = true;
        current_state.mode = "OFFBOARD";

        if(tasks.size()==0)
        {
          needsToLand = true;
          countToLanding++;
          if(countToLanding>80)
          {
             offb_set_mode.request.custom_mode = "AUTO.LAND";
             set_mode_client.call(offb_set_mode);
             break;
          }
        }

        if(!needsToLand && armed && (ros::Time::now() - start_scenario>ros::Duration(tasks.front().Time())) && current_state.mode == "OFFBOARD" )
        {
            waypoint currentWaypoint = tasks.front();
            tasks.pop();
            posTarget.position.x = currentWaypoint.X();
            posTarget.position.y = currentWaypoint.Y();
            posTarget.position.z = currentWaypoint.Z();
            posTarget.yaw        = static_cast<float>(currentWaypoint.Yaw());

            ROS_INFO("Current time is %d ,going to %lf,%lf,%lf,%lf",(ros::Time::now() - start_scenario).sec,posTarget.position.x,posTarget.position.y,posTarget.position.z,posTarget.yaw);
        }

        pos_Target_pub.publish(posTarget);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
