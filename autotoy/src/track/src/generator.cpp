#include "ros/ros.h"
#include "track/Generator.h"
#include "track/ConePlacer.h"
#include "math.h"

/* This script defines the functionality of the generator node which in essence 
creates a client which requests a service to conesplacer by passing as request a generated centerline 
and then creates a response which is sent to Simulator as part of the Generator service it provides.
*/


class Generator {
public:

  ros::NodeHandle n;
  ros::ServiceClient client;
  ros::ServiceServer service;
  
  void start() {
    service = n.advertiseService("/track/generate", &Generator::generate_track, this);
  }

  bool generate_track(track::Generator::Request& req, track::Generator::Response& res) //always has to be a bool
  {
    // Create centerline
    ROS_INFO("Creating Centerline");
    std::vector<track::Point> centerline_points;
    track::Line centerline_line;
    
    // Create points of the centerline and append them to the centerline vector

    
    track::Point new_point;
  
    float theta = 0;
    int h = 0;
    int k = 10;
    int r = 10;
    float step=15*M_PI/180;

    while (theta < 2 * M_PI) {
      new_point.x = (float) (h + r*cos(theta));
      new_point.y = (float) (k + r*sin(theta));

      centerline_points.push_back(new_point);
      theta+=step;
    }

    
    centerline_line.points = centerline_points;

    // Create client
    ros::ServiceClient client = n.serviceClient<track::ConePlacer>("/track/coneplacer");
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
      ROS_ERROR("Failed to call the service /track/coneplacer");
      return false;
    }
  }
};



int main(int argc, char **argv)
{
  // Init the node
  ROS_INFO("Starting 'generator' node...");
  ros::init(argc, argv, "generator");

  // Instantiate a Generator object
  Generator generator;

  generator.start();

  ROS_INFO("Ready to generate track.");

  ros::spin();
  return 0;
}



