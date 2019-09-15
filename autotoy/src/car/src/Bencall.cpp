#include "ros/ros.h"
#include "track/Generator.h"

int main(int argc, char **argv)
{
  // Initialize ROS; name of the node: "generator"
  ros::init(argc, argv, "Benca");
  ros::NodeHandle n;

  // ros::ServiceClient client = n.serviceClient<PACKAGE_NAME::SERVICE_NAME>("SERVICE_NAME");
  // create client needed to request from ben
  ros::ServiceClient client = n.serviceClient<track::Generator>("/track/Generator"); 

  ROS_INFO("Ready to call Ben.");

  track::Generator srv;

  if (client.call(srv))
  {
    ROS_INFO("requested cones from centreline generator");
  }
  else
  {
    ROS_ERROR("Failed to call service from coneplacer");
  }

  for (int i=0; i<srv.response.track.leftcones.cones.size(); i++){
    ROS_INFO("%f, %f", srv.response.track.leftcones.cones[i].x, srv.response.track.leftcones.cones[i].y);
  }

  return 0;
}