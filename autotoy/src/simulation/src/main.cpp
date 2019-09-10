#include "ros/ros.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "simulation-main");
  ros::NodeHandle n;
  ros::spin();
  return 0;
}