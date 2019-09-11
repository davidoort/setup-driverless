#include "ros/ros.h"
#include <math.h>
#include <iostream>
#include <vector>
#include <tuple>


std::tuple<std::vector<std::vector<float>>,std::vector<std::vector<float>>>  conePlacer(const std::vector<std::vector<float>>  &centreline, const float conedistance) //centreline is a 2D std::vector of points (x,y)
//returns 2 std::vector objects in the same shape for leftcones and right cones
{
  const int length =  centreline.size(); //find length of centreline array
  std::vector<std::vector<float>> leftcones(length, std::vector<float>(2,0)), rightcones(length, std::vector<float>(2,0)); //create return arrays each an array of two points
  
  //init values for temporary calculations in for loop
  std::vector<float> direction(2); //initalise direction vector for each point on the centre line
  float directionmag; //magnitude of direcvector
  std::vector<float> leftconedirection(2);

  for(int i = 0; i < length; i++)
  {
    // check if at beginning, middle, or end of the array
    if(i == 0) //first point
    {
      direction[0] = centreline[1][0] - centreline[0][0]; //change in x
      direction[1] = centreline[1][1] - centreline[0][1]; //change in y
    }

    else if(i == length-1) //final point
    {
      direction[0] = centreline[length][0] - centreline[length-1][0]; //change in x
      direction[1] = centreline[length][1] - centreline[length-1][1]; //change in y
    }

    else //middle point
    {
      direction[0] = centreline[i+1][0] - centreline[i-1][0]; //change in x
      direction[1] = centreline[i+1][1] - centreline[i-1][1]; //change in y
    }

  directionmag = sqrt(direction[0]*direction[0] + direction[1]*direction[1]); //find magnitude of direction vector
  leftconedirection = {-direction[1]/directionmag, direction[0]/directionmag}; //rotate direction vector 90 degrees counter clockwise and convert to unit vector
  
  leftcones[i] = {centreline[i][0] + leftconedirection[0]*conedistance, centreline[i][1] + leftconedirection[1]*conedistance}; //find displacement from centreline point for left cone
  rightcones[i] = {centreline[i][0] - leftconedirection[0]*conedistance, centreline[i][1] - leftconedirection[1]*conedistance}; //find displacement from centreline point for the right cone

  return {leftcones, rightcones};    
  }
}

struct Cones
{
  std::vector<std::vector<float>> leftcones;// = new std::vector<std::vector<float>>;
  std::vector<std::vector<float>> rightcones;// = new std::vector<std::vector<float>>;
};



int main()//int argc, char **argv)
{
  // ros::init(argc, argv, "coneplacer");
  // ros::NodeHandle n;
  // ros::spin();

  //example line
  std::vector<std::vector<float>> line(10, std::vector<float>(2,0));

  float holderx = 0;
  float holdery = 0;
  for(int i = 0; i < line.size(); i++)
  {
    line[i][0] = holderx;
    line[i][1] = holdery;
    holderx += 2;
    holdery += 1;
  }


  //Cones* result = new Cones;
  Cones result = conePlacer(line, 0.2);
  //std::cout << (*leftcones)[0][1] << std::endl;
  delete result;

  return 0;
}