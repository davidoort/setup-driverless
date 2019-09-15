#include "ros/ros.h"
#include "track/Generator.h"
#include "car/Location.h"
#include "car/Control.h"
#include <tuple>

/* 
This script is supposed to define the functionality of the "GodNode" which is the simulator. 

- First of all, it starts the simulation by defining a client of the "generate_track" service which returns a track. 
- It then has to publish the location of the car to the car_state topic and the location (and color) of the cones to the cone_world topic
- It will subscribe to car_control topic to receive control command messages, which will be passed through a car model and the car location will be updated.
- It will subscribe to a bunch of extra topics such as visible_cones, target_path to be able to represent these things in Gazebo or Rviz and help with debugging
*/


// Define Car object (for the model)

class Car {
public:

  float x; // m
  float y; // m
  float heading; // rad
  float velocity; // m/s


  Car()
  : x(0.0), y(0.0), heading(0.0) {}

  void move(const float accel, const float yawrate, const float dt) {


    velocity = velocity + accel*dt;

    x = x + velocity*dt*cos(heading);
    y = y + velocity*dt*sin(heading);

    heading = yawrate*dt;
  
  }


};


class Simulator {
public:

  ros::ServiceClient client;
  ros::NodeHandle nodeHandle;
  ros::Publisher cones_pub, car_location_pub;
  ros::Subscriber car_cmd_sub, visible_cones_sub, target_path_sub;

  std::tuple<track::Line, track::Cones> getTrack() {
    client = nodeHandle.serviceClient<track::Generator>("generate_track");
    track::Generator srv;
    if (client.call(srv)){
      ROS_INFO("Received a track!");
      // Extract a centerline from the track message

      track::Line centerline = srv.response.track.centreline;

      // Extract cones (to be published later) from the track message

      track::Cones cones = srv.response.track.cones;
      return std::make_tuple(centerline, cones);
    }
    else
    {
      ROS_ERROR("Failed to call service generate_track");
      ros::shutdown(); //  this shuts down all publishers (and everything else).
    }

  }

  void publish_Cones(track::Cones& cones) {
    ROS_INFO("Publishing cones!");
    cones_pub = nodeHandle.advertise<track::Cones>("cone_world", 1, true);
    cones_pub.publish(cones);
  };


  // Callbacks

  void Simulator::command_callback(const ros::MessageEvent<car::Control const>& event)
  {
    /* const std::string &publisher_name = event.getPublisherName();
    const ros::M_string &header = event.getConnectionHeader();
    

    const car::Control &msg = event.getMessage(); */

    ros::Time receipt_time = event.getReceiptTime();
    ROS_INFO("Received control command at time %f", receipt_time);

  }

  void Simulator::visible_cone_callback(track::Cones& visible_cones) {

    // Show these cones in Gazebo
    ROS_INFO("Received visible cones!");
  }

  void Simulator::target_path_callback(track::Line& target_path) {


    ROS_INFO("Received target path!");
    
  }



};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "GodNode");

  Simulator simulator;

  track::Line centerline;
  track::Cones cones;

  tie(centerline, cones) = simulator.getTrack();

  // Do something with centerline, like publish it on a visualization topic

  // Do something with cones, like visualization or more important, publish them on the cone_world topic

  simulator.publish_Cones(cones);

  // Initialize publishers and subscribers
  simulator.car_location_pub = simulator.nodeHandle.advertise<car::Location>("car_state", 100);

  simulator.car_cmd_sub = simulator.nodeHandle.subscribe("car_control", 100, &Simulator::command_callback, &simulator); // Not sure if the first & is needed
  simulator.visible_cones_sub = simulator.nodeHandle.subscribe("visible_cones", 100, Simulator::visible_cone_callback, &simulator);
  simulator.target_path_sub = simulator.nodeHandle.subscribe("target_path", 100, Simulator::target_path_callback, &simulator);

  ros::Rate loop_rate(10);
  
  // car location initialized at x=y=theta=0
  Car car;

  int count = 0;
  while (ros::ok())
  {
   

    // Compile the Location.msg message


    car::Location location;

    location.location.x = car.x;
    location.location.y = car.y;
    location.heading = car.heading;

    simulator.car_location_pub.publish(location);

    ros::spinOnce();
    loop_rate.sleep();
    ++count;

    ROS_INFO("%d", count);

  }

  
  return 0;

}