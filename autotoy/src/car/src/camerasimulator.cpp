#include "ros/ros.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "camerasimulator");
  ros::NodeHandle n;
  ros::spin();
  return 0;
}