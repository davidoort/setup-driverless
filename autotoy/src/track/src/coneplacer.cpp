#include "ros/ros.h"
#include "track/ConePlacer.h"
#include "track/Track.h" //includes both the lines and cones object
// #include "track/Cones.h"
// #include "track/Line.h"
#include <math.h>
#include <iostream>

struct ConesInternal
{
  track::Cones leftcones;
  track::Cones rightcones;
};

struct PointInternal
{
  float x;
  float y;
};

void  conePlacer(const track::Line &centreline, const float conedistance, ConesInternal &conesobj) //centreline is line class from generator, conedistance describes cones displacement from centre
// conesobj is reference to temporary holder for cone arrays || cones are changed by reference so no return values
{
  std::cout << "conePlacer started" << std::endl;
  const int length =  centreline.points.size(); //find length of centreline array
  
  //init values for temporary calculations in for loop
  std::vector<float> direction = {(float)0,(float)0}; //initalise direction vector for each point on the centre line
  float directionmag; //magnitude of direcvector
  std::vector<float> leftconedirection = {(float)0,(float)0};

  track::Point* leftpoint = new track::Point; // to store temporary point for left cones
  track::Point* rightpoint = new track::Point; // to store temporary point for right cones

  for(int i = 1; i < length; i++)
  {
    // check if at beginning, middle, or end of the array
    if(i == 0) //first point
    {
      direction[0] = centreline.points[1].x - centreline.points[0].x; //change in x
      direction[1] = centreline.points[1].y - centreline.points[0].y; //change in y
    }

    else if(i == length-1) //final point
    {
      direction[0] = centreline.points[length].x - centreline.points[length-1].x; //change in x
      direction[1] = centreline.points[length].y - centreline.points[length-1].y; //change in y
    }

    else //middle point
    {
      direction[0] = centreline.points[i+1].x - centreline.points[i-1].x; //change in x
      direction[1] = centreline.points[i+1].y- centreline.points[i-1].y; //change in y
    }

    directionmag = (float)(sqrt(direction[0]*direction[0] + direction[1]*direction[1])); //find magnitude of direction vector
    leftconedirection[0] = -(direction[1])/directionmag;
    leftconedirection[1] = (direction[0])/directionmag;
    // leftconedirection = {-direction[1]/directionmag, direction[0]/directionmag}; //rotate direction vector 90 degrees counter clockwise and convert to unit vector
    
    leftpoint->x = (float)(centreline.points[i].x + leftconedirection[0]*conedistance); // push x,y points of left cone to temporary point object holder
    leftpoint->y = (float)(centreline.points[i].y + leftconedirection[1]*conedistance);

    rightpoint->x = (float)(centreline.points[i].x - leftconedirection[0]*conedistance); // push x,y points of right cone to temporary point object holder
    rightpoint->y = (float)(centreline.points[i].x - leftconedirection[1]*conedistance);
    

    conesobj.leftcones.cones.push_back(*leftpoint); //append leftpoint to cones obj
    conesobj.rightcones.cones.push_back(*rightpoint); //append rightpoint to cones obj

    
  }

  delete leftpoint, rightpoint;
  std::cout << "Cones Calculated" << std::endl;
}

bool server(track::ConePlacer::Request  &req, track::ConePlacer::Response &res)
{
  ConesInternal* conesptr = new ConesInternal; // add ConesInternal to the heap (where can I delete it?)
  conePlacer(req.centreline, (float)1.5, *conesptr); // fill Cones objects in conesptr object
  res.leftcones = (*conesptr).leftcones; // fill response with left cones
  res.rightcones = (*conesptr).rightcones; // fille response with rightcones
  ROS_INFO("Generated cone placements in response to ConePlacer-request");
  return true;
}
        


int main(int argc, char **argv)
{
  ros::init(argc, argv, "coneplacer"); //initialise node
  ros::NodeHandle n; //node variable

  ros::ServiceServer service = n.advertiseService("/track/coneplacer", server); // create server for coneplacer
  ROS_INFO("Ready to generate cones.");
  ros::spin();

  return 0;
}