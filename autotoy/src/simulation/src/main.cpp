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




};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "GodNode");

  Simulator simulator;

  track::Line centerline;
  track::Cones cones;

  tie(centerline, cones) = simulator.getTrack();

  ros::spin();
  
  return 0;

}