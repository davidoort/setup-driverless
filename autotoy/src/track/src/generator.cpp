#include "ros/ros.h"
#include "track/Generator.h"
#include "track/ConePlacer.h"
#include "track/Track.h"
// to use a client node within a server node go to https://answers.ros.org/question/197868/can-a-node-be-a-subscriber-and-client-at-the-same-time/


//declare some variables that need to be defined before generateTrack is defined
bool generateTrack(track::Generator::Request  &req, track::Generator::Response &res);
ros::ServiceClient *clientPtr; //pointer for a client



int main(int argc, char **argv)
{
  ros::init(argc, argv, "generator");
  ros::NodeHandle n;

  ros::ServiceClient client = n.serviceClient<track::ConePlacer>("/track/coneplacer"); //create client needed to request from cones
  clientPtr = &client; //give the address of the client to the clientPtr

  ros::ServiceServer service = n.advertiseService("/track/generate", generateTrack);

  ROS_INFO("Ready to generate tracks.");
  ros::spin();

  return 0;
}

bool generateTrack(track::Generator::Request  &req,
         track::Generator::Response &res)
{
  // create centreline
  int numberofpoints = 5;
  std::vector<track::Point> my_array(numberofpoints);
  for(int i = 0; i < numberofpoints; i++)
  {
    my_array[i].x = (float)(i*1+3);
    my_array[i].y = (float)(i*2+4);
  }

  std::vector<track::Point> centrelinepoints = my_array;
  track::Line* centreline = new track::Line();
  centreline->points = centrelinepoints;

  //set up client made in main and request data from ConePlacer
  ros::ServiceClient client = (ros::ServiceClient)*clientPtr; //dereference the clientPtr
  track::ConePlacer srv;
  srv.request.centreline = *centreline;
  if (client.call(srv))
  {
    ROS_INFO("requested cones from centreline generator");
  }
  else
  {
    ROS_ERROR("Failed to call service from coneplacer");
  }


  //create and fill node response object (track)
  track::Track* output = new track::Track();
  output->centreline = *(centreline);
  output->leftcones = srv.response.leftcones;
  output->rightcones = srv.response.rightcones;
  res.track = *output;
  ROS_INFO("Generated new track in response to generator-request");
  return true;
}
