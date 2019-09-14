#include "ros/ros.h"
#include "track/ConePlacer.h"

/* 
This script is supposed to advertise the cone_placer service and contain the function that takes in the 
service request (centerline) and outputs a response (cones).
*/

bool cone_placer(track::ConePlacer::Request& req, track::ConePlacer::Response& res)
{
  // Declare variables
  track::Cones placed_cones;
  std::vector<track::Cone> cone_vector;
  track::Cone new_cone_left, new_cone_right;


  // Do your cone placement -this example shows some ways of manipulating the request and response variables

  std::vector<track::Point> centreline_points = req.centreline.points;

  int i;
  for(i=0; i<centreline_points.size(); i++)
  {
    
    new_cone_left.color = 0; //left cone
    new_cone_left.position.x = centreline_points[i].x+1;
    new_cone_left.position.y = centreline_points[i].y-1;
  
    new_cone_right.color = 1; //right cone
    new_cone_right.position.x = centreline_points[i].x-1;
    new_cone_right.position.y = centreline_points[i].y+1;


    cone_vector.push_back(new_cone_left);
    cone_vector.push_back(new_cone_right);
  }

  placed_cones.cones = cone_vector;

  // return placed cones
  res.cones = placed_cones;

  ROS_INFO("Cones placed!");

  return true;

}


int main(int argc, char **argv)
{
  ROS_INFO("Starting 'coneplacer' node...");
  ros::init(argc, argv, "coneplacer");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("place_cones", cone_placer);

  ros::spin();
  return 0;
}