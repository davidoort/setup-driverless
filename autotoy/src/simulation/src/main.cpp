#include "ros/ros.h"
#include "track/Generator.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "simulationmain");
  ros::NodeHandle n;

  ros::ServiceClient client = n.serviceClient<track::Generator>("/track/generate");
  track::Generator srv;
  if (client.call(srv)){
    ROS_INFO("Track start point: %f;%f", srv.response.track.centreline.points[0].x, srv.response.track.centreline.points[0].y);
  }
  else
  {
    ROS_ERROR("Failed to call service /track/generate");
    return 1;
  }

  ros::spin();
  return 0;
} 