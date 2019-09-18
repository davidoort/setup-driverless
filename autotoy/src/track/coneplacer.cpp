#include "ros/ros.h"
#include "track/ConePlacer.h"
#include <math.h>
#include <iostream>

/* 
This script is supposed to advertise the cone_placer service and contain the function that takes in the 
service request (centerline) and outputs a response (cones).
*/

//code to take in centreline points and produce the coneobjects
void  conePlacer(const std::vector<track::Point> &centreline, std::vector<track::Cone> &cone_vector, const float conedistance) //centreline is line class from generator, conedistance describes cones displacement from centre
// conesobj is reference to temporary holder for cone arrays || cones are changed by reference so no return values
{
  std::cout << "conePlacer started" << std::endl;
  const int length =  centreline.size(); //find length of centreline array
  
  //init values for temporary calculations in for loop
  std::vector<float> direction = {(float)0,(float)0}; //initalise direction vector for each point on the centre line
  float directionmag; //magnitude of direcvector
  std::vector<float> leftconedirection = {(float)0,(float)0};

  track::Cone* new_cone_left = new track::Cone; // to store temporary point for left cones
  track::Cone* new_cone_right = new track::Cone; // to store temporary point for right cones

  for(int i = 0; i < length; i++)
  {
    // check if at beginning, middle, or end of the array
    if(i == 0) //first point
    {
      direction[0] = centreline[1].x - centreline[0].x; //change in x
      direction[1] = centreline[1].y - centreline[0].y; //change in y
    }

    else if(i == length-1) //final point
    {
      direction[0] = centreline[length-1].x - centreline[length-2].x; //change in x
      direction[1] = centreline[length-1].y - centreline[length-2].y; //change in y
    }

    else //middle point
    {
      direction[0] = centreline[i+1].x - centreline[i-1].x; //change in x
      direction[1] = centreline[i+1].y- centreline[i-1].y; //change in y
    }

    directionmag = (float)(sqrt(direction[0]*direction[0] + direction[1]*direction[1])); //find magnitude of direction vector
    leftconedirection[0] = -(direction[1])/directionmag;
    leftconedirection[1] = (direction[0])/directionmag;
    // leftconedirection = {-direction[1]/directionmag, direction[0]/directionmag}; //rotate direction vector 90 degrees counter clockwise and convert to unit vector
    
    new_cone_left->position.x = (float)(centreline[i].x + leftconedirection[0]*conedistance); // push x,y points of left cone to temporary point object holder
    new_cone_left->position.y = (float)(centreline[i].y + leftconedirection[1]*conedistance);
    new_cone_left->color = 0;

    new_cone_right->position.x = (float)(centreline[i].x - leftconedirection[0]*conedistance); // push x,y points of right cone to temporary point object holder
    new_cone_right->position.y = (float)(centreline[i].y - leftconedirection[1]*conedistance);
    new_cone_right->color = 1;

    cone_vector.push_back(*new_cone_left); //append leftpoint to cones obj
    cone_vector.push_back(*new_cone_right); //append rightpoint to cones obj

    
  }

  delete new_cone_left, new_cone_right;
  std::cout << "Cones Calculated" << std::endl;
}


bool cone_placer(track::ConePlacer::Request& req, track::ConePlacer::Response& res)
{
  // Declare variables
  float conedistance = 1.5;
  track::Cones* placed_cones = new track::Cones;
  std::vector<track::Cone>* cone_vector = new std::vector<track::Cone>;
  std::vector<track::Point> centreline_points = req.centreline.points;
  // track::Cone new_cone_left, new_cone_right;


  // Do your cone placement -this example shows some ways of manipulating the request and response variables

  conePlacer(centreline_points, *cone_vector, conedistance);

  // int i;
  // for(i=0; i<centreline_points.size(); i++)
  // {
    
  //   new_cone_left.color = 0; //left cone
  //   new_cone_left.position.x = centreline_points[i].x+1;
  //   new_cone_left.position.y = centreline_points[i].y-1;
  
  //   new_cone_right.color = 1; //right cone
  //   new_cone_right.position.x = centreline_points[i].x-1;
  //   new_cone_right.position.y = centreline_points[i].y+1;


  //   cone_vector.push_back(new_cone_left);
  //   cone_vector.push_back(new_cone_right);
  // }

  placed_cones->cones = *cone_vector;

  // return placed cones
  res.cones = *placed_cones;
  delete placed_cones;
  delete cone_vector;

  ROS_INFO("Cones placed!");

  return true;

}


int main(int argc, char **argv)
{
  ROS_INFO("Starting 'coneplacer' node...");
  ros::init(argc, argv, "coneplacer");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("/track/coneplacer", cone_placer);

  ros::spin();

  return 0;
}
