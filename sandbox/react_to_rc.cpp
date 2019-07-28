#include <ros/ros.h>
#include <mavros_msgs/RCIn.h>

void Rc_in_callback(const mavros_msgs::RCIn rcIn)
{
    ROS_INFO("Hello world!");
    ROS_INFO("%f",rcIn.channels[1]);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "react_to_rc");
  ros::NodeHandle nh;

  ros::Rate rate(20.0);
  ros::Subscriber rc_in_sub = nh.subscribe("/mavros/rc/in",50, Rc_in_callback);
  while (ros::ok()) {

    ros::spinOnce();
    rate.sleep();
  }

}
