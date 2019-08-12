#include <ros/ros.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>

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


int main(int argc, char **argv)
{

  std::vector<std::vector<double>> table;
  readFile("/home/makeruser/catkin_ws/src/sandbox/route.txt",table);

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
