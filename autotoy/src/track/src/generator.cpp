#include "ros/ros.h"
#include "track/Generator.h"
#include "track/Track.h"

bool add(track::Generator::Request  &req,
         track::Generator::Response &res)
{
  std::vector<track::Point> centrelinepoints = {*(new track::Point()), *(new track::Point())};
  track::Line* centreline = new track::Line();
  centreline->points = centrelinepoints;

  track::Track* output = new track::Track();
  output->centreline = *(centreline);
  res.track = *output;
  ROS_INFO("Generated new track in response to generator-request");
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "generator");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("/track/generate", add);
  ROS_INFO("Ready to generate tracks.");
  ros::spin();

  return 0;
} 