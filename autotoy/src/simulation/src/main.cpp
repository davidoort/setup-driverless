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

// in seconds
#define timeBetweenTick 1/10

class Car {
public:

  float x; // meter
  float y; // meter
  float heading; // radians
  float velocity; // meter/second


  Car() : x(0.0), y(0.0), heading(0.0) {}

  void move(const float accel, const float yawrate) {
    velocity = velocity + accel*timeBetweenTick;

    x = x + velocity*timeBetweenTick*cos(heading);
    y = y + velocity*timeBetweenTick*sin(heading);

    heading = yawrate*timeBetweenTick;
  }
};


class Simulator {
public:
  ros::Publisher cones_pub, car_location_pub;
  ros::Subscriber car_cmd_sub, visible_cones_sub, target_path_sub;
  Car car;

  std::tuple<track::Line, track::Cones> getTrack(ros::NodeHandle& nodeHandle) {
    ros::ServiceClient client = nodeHandle.serviceClient<track::Generator>("generate_track");
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

  void publish_Cones(ros::NodeHandle& nodeHandle, track::Cones& cones) {
    ROS_INFO("Publishing cones!");
    cones_pub = nodeHandle.advertise<track::Cones>("cone_world", 1, true);
    cones_pub.publish(cones);
  };


  // Callbacks

  void controlCommandReceived(const car::Control& control)
  {
    car.move(control.acceleration, control.yawrate);
    ROS_INFO("Received control command");
  }

  void visableConesReceived(const track::Cones& visible_cones) {

    // Show these cones in Gazebo
    ROS_INFO("Received visible cones!");
  }

  void targetPathReceived(const track::Line& target_path) {
    ROS_INFO("Received target path!");
  }

  void setCar(Car& newCar) {
    car = newCar;
  }
};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "GodNode");
  ros::NodeHandle nodeHandler;

  Car car;
  Simulator simulator;
  simulator.setCar(car);

  track::Line centerline;
  track::Cones cones;

  tie(centerline, cones) = simulator.getTrack(nodeHandler);

  // Do something with centerline, like publish it on a visualization topic

  // Do something with cones, like visualization or more important, publish them on the cone_world topic

  simulator.publish_Cones(nodeHandler, cones);

  // Initialize publishers and subscribers
  simulator.car_location_pub = nodeHandler.advertise<car::Location>("car_state", 100);

  simulator.car_cmd_sub = nodeHandler.subscribe("car_control", 100, &Simulator::controlCommandReceived, &simulator); // Not sure if the first & is needed
  simulator.visible_cones_sub = nodeHandler.subscribe("visible_cones", 100, &Simulator::visableConesReceived, &simulator);
  simulator.target_path_sub = nodeHandler.subscribe("target_path", 100, &Simulator::targetPathReceived, &simulator);

  ros::Rate loop_rate(1/timeBetweenTick);
  
  // car location initialized at x=y=theta=0

  while (ros::ok())
  {
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