#include <ros/ros.h>

int main(int argc, char **argv)
{

  ROS_INFO("argc = ! %s ",argv[0]);
  if(argc>1)
  {
     ROS_INFO("-------------------Hello world! %d ",atoi(argv[1])+1);
  }


   ros::init(argc, argv, "test1");
   ros::NodeHandle nh;

   ros::Rate rate(20.0);

   ROS_INFO("Hello world!");

}
