#include "ros/ros.h"
#include "track/Generator.h"
#include "track/Cone.h"
#include "track/Cones.h"
#include "track/Line.h"
#include <array>


int main(int argc, char **argv)
{
  // Initialize ROS; name of the node: "generator"
  ros::init(argc, argv, "visualize_track");
  ros::NodeHandle n;

  // ros::ServiceClient client = n.serviceClient<PACKAGE_NAME::SERVICE_NAME>("SERVICE_NAME");
  // create client needed to request from ben
  // ros::ServiceClient client = n.serviceClient<track::Generator>("/track/Generator"); 
  ros::ServiceClient client = n.serviceClient<track::Generator>("/track/generate"); 

  ROS_INFO("Ready to call generator service.");

  track::Generator srv;

  if (client.call(srv))
  {
    ROS_INFO("Requested cones from centreline generator");
  }
  else
  {
    ROS_ERROR("Failed to call service from coneplacer");
  }

  int size = srv.response.track.cones.cones.size();
  //int size = sizeof(srv.response) / (2*sizeof(float32)+ sizeof(bool))
  for (int i=0; i<size; i++){
    ROS_INFO("%f, %f, %i", srv.response.track.cones.cones[i].position.x, srv.response.track.cones.cones[i].position.y, srv.response.track.cones.cones[i].color);
  }

  return 0;
}