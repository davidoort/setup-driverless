#include <opencv2/core.hpp>
#include <opencv2/ml.hpp>
#include "ros/ros.h"
#include "track/Cone.h"
#include "track/Cones.h"
#include "track/Line.h"


int main(int argc, char **argv) {

  // Init node with name 'trackfinder'
  ros::init(argc, argv, "test_svm");

  ros::NodeHandle n;
  ros::Publisher send_track = n.advertise<track::Cones>("/car/cameratest", 1000);


  int num_points = 10;
  float power = 1.25;

  int a[10];

  std::vector<track::Cone> new_cones;
  track::Cones send_cones;

  // track::Cone new_cones[num_points*2];
  ROS_INFO("Generating fake track...");

  for(int i=0; i<num_points; i++){

    track::Cone coneleft;
    track::Cone coneright;

    coneleft.position.x = pow(i, power);
    coneleft.position.y = i;
    coneleft.color = 0;

    coneright.position.x = pow(i, power) + 10;
    coneright.position.y = i;
    coneright.color = 1;

    new_cones.push_back(coneleft);
    new_cones.push_back(coneright);

    ROS_INFO("%f, %f", coneleft.position.x, coneleft.position.y);
  }

  ROS_INFO("Generated fake track");

  send_cones.cones = new_cones;
  ros::Rate loop_rate(0.3);

  while(ros::ok()){
    send_track.publish(send_cones);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
