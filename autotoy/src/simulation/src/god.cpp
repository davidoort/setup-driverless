#include "ros/ros.h"
#include "track/Generator.h"
#include "car/Location.h"
#include "car/Control.h"
#include <tuple>
#include <visualization_msgs/Marker.h>
#include <cmath>

/* 
This script is supposed to define the functionality of the "GodNode" which is the simulator. 

- First of all, it starts the simulation by defining a client of the "/track/generate" service which returns a track. 
- It then has to publish the location of the car to the /car/location topic and the location (and color) of the cones to the /track/cones topic
- It will subscribe to /car/location topic to receive control command messages, which will be passed through a car model and the car location will be updated.
- It will subscribe to a bunch of extra topics such as car/camera, car/targetline to be able to represent these things in RViz and help with debugging
*/



// in seconds
#define timeBetweenTick 0.1

class Car {
public:

  float x; // meter
  float y; // meter
  float heading; // radians
  float velocity; // meter/second
  float acceleration; // meter/second^2
  float yawrate; // radians/second

  Car() : x(0.0), y(0.0), heading(0.0), velocity(0.0), acceleration(0.0), yawrate(0.0) {}

  void move() {
    velocity = velocity + acceleration*timeBetweenTick;
    heading = heading + yawrate*timeBetweenTick;
    x = x + velocity*timeBetweenTick*cos(heading);
    y = y + velocity*timeBetweenTick*sin(heading);
  }

  void control(const float accel, const float newYawrate) {
    yawrate = newYawrate;
    acceleration = accel;
  }
};


class Simulator {
public:
  ros::Publisher cones_pub, car_location_pub;
  ros::Subscriber car_cmd_sub, visible_cones_sub, target_path_sub;
  Car* car;

  std::tuple<track::Line, track::Cones> getTrack(ros::NodeHandle& nodeHandle) {
    ros::ServiceClient client = nodeHandle.serviceClient<track::Generator>("/track/generate");
    track::Generator srv;
    if (client.call(srv)){
      ROS_INFO("Received a track!");

      // Extract a centerline from the track message
      track::Line centerline = srv.response.track.centreline;

      // Extract cones (to be published later) from the track message
      track::Cones cones = srv.response.track.cones;

      return std::make_tuple(centerline, cones);
    }

    ROS_ERROR("Failed to call service /track/generate");
    ros::shutdown(); //  this shuts down all publishers (and everything else).
    return std::make_tuple(track::Line(), track::Cones());
  }

  void publish_Cones(ros::NodeHandle& nodeHandle, track::Cones& cones) {
    ROS_INFO("Publishing cones!");
    cones_pub = nodeHandle.advertise<track::Cones>("/track/cones", 1, true);
    cones_pub.publish(cones);
  };



  void controlCommandReceived(const car::Control& control)
  {
    car->control(control.acceleration, control.yawrate);
    ROS_INFO("Received control command!");
  }

  void visibleConesReceived(const track::Cones& visible_cones) {

    // Show these cones in RViz
    ROS_INFO("Received visible cones!");
  }

  void targetPathReceived(const track::Line& target_path) {
    
    // Show these cones in RViz
    ROS_INFO("Received target path!");
  }

  void setCar(Car* newCar) {
    car = newCar;
  }
};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "god");
  ros::NodeHandle nodeHandler;
  ros::Publisher marker_pub = nodeHandler.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  ros::Rate r(30);

  Car car;
  Simulator simulator;
  simulator.setCar(&car);

  track::Line centerline;
  track::Cones cones;

  tie(centerline, cones) = simulator.getTrack(nodeHandler);

  visualization_msgs::Marker points;
  points.header.frame_id = "/cones";
  points.header.stamp = ros::Time::now();
  points.ns = "track_cones_location";
  points.action = visualization_msgs::Marker::ADD;
  points.id = 0;
  points.type = visualization_msgs::Marker::POINTS;

  points.scale.x = 0.2;
  points.scale.y = 0.2;
  points.color.g = 1.0f;
  points.color.a = 1.0;
  int z = 0;


  int size = cones.cones.size();
  for (int i=0; i<size; i++){
    ROS_INFO("%f, %f, %i", cones.cones[i].position.x, cones.cones[i].position.y, cones.cones[i].color);

    geometry_msgs::Point p;
    p.x = cones.cones[i].position.x;
    p.y = cones.cones[i].position.y;
    p.z = z;

    points.points.push_back(p);

    ROS_INFO("P");
  }

  marker_pub.publish(points);
  // Do something with centerline, like publish it on a visualization topic
  // ros::Publisher centerline_pub = nodeHandler.advertise<track::Line>("/viz/centerline", 100);
  // Do something with cones, like visualization or more important, publish them on the cone_world topic
  
  simulator.publish_Cones(nodeHandler, cones);

  // Initialize publishers and subscribers
  simulator.car_location_pub = nodeHandler.advertise<car::Location>("/car/location", 100);

  simulator.car_cmd_sub = nodeHandler.subscribe("/car/controls", 100, &Simulator::controlCommandReceived, &simulator); // Not sure if the first & is needed
  simulator.visible_cones_sub = nodeHandler.subscribe("/car/camera", 100, &Simulator::visibleConesReceived, &simulator);
  simulator.target_path_sub = nodeHandler.subscribe("/car/targetline", 100, &Simulator::targetPathReceived, &simulator);

  ros::Rate loop_rate(1/timeBetweenTick);


  while (ros::ok())
  {
    car.move();
    car::Location location;
    location.location.x = car.x;
    location.location.y = car.y;
    location.heading = car.heading;

    simulator.car_location_pub.publish(location);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}