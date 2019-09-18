#include <opencv2/core.hpp>
#include <opencv2/ml.hpp>
#include "ros/ros.h"
#include "track/Cone.h"
#include "track/Cones.h"
#include "track/Line.h"
#include "car/Location.h"


int main(int argc, char **argv) {

  // Init node with name 'trackfinder'
  ros::init(argc, argv, "test_orderdata");
  ros::NodeHandle n;

  ros::Publisher send_carloc = n.advertise<car::Location>("/car/locationtest", 1000);


  ros::Rate loop_rate(10);
  // send_camera.publish(send_cones);

  car::Location location;
  location.location.x = 30;
  location.location.y = 0;
  location.heading = 0;

  while(ros::ok()){
    location.location.y += 1;
    send_carloc.publish(location);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}