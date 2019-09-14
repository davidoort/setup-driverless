#include "ros/ros.h"
#include "track/Generator.h"
#include "track/ConePlacer.h"

/* This script defines the functionality of the generator node which in essence 
creates a client which requests a service to conesplacer by passing as request a generated centerline 
and then creates a response which is sent to Simulator as part of the Generator service it provides.
*/

ros::NodeHandle n;


bool generate_track(track::Generator::Request& req, track::Generator::Response& res) //always has to be a bool
{
  //track::Line centerline = track::Point[3];
  //std::vector<track::Point> centerline

  // Create centerline
  std::vector<track::Point> centerline_points;
  track::Line centerline_line;
  
  // Create points of the centerline and append them to the centerline vector

  int i;
  for(i=0; i<5; i++)
  {
    track::Point new_point;
    new_point.x = (float)i;
    new_point.y = (float)i;

    centerline_points.push_back(new_point);
  }
  
  centerline_line.points = centerline_points;

  // Create client
  ros::ServiceClient client = n.serviceClient<track::ConePlacer>("place_cones");
  track::ConePlacer srv;
  srv.request.centreline = centerline_line;

  if(client.call(srv))
  {
    // The call goes through and a response is received
    ROS_INFO("Cones placed!");

    track::Cones placed_cones =  srv.response.cones;
    track::Track track;
    track.centreline = centerline_line;
    track.cones = placed_cones;
    res.track = track;

    ROS_INFO("Track generated!");

    return true;
  }
  else
  {
    ROS_ERROR("Failed to call the service place_cones");
    return false;
  }
  
}




int main(int argc, char **argv)
{
  ros::init(argc, argv, "generator_server");

  ros::ServiceServer service = n.advertiseService("generate_track", generate_track);
  
  ROS_INFO("Ready to generate track.");
  ros::spin();

  return 0;
}

