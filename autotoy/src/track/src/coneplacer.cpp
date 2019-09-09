#include "ros/ros.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "coneplacer");
  ros::NodeHandle n;
  ros::spin();
  return 0;
}