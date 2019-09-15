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
  ros::Publisher send_camera = n.advertise<track::Cones>("/car/camera", 1000);

  int n_cones = 8;
  float cones_x[n_cones] = {100,100,200,300,300,400,50,250};
  float cones_y[n_cones] = {150,450,300,150,450,300,50,50};
  int cones_col[n_cones] = { 1, 1, 1, 0, 0, 0, 1, 0};

  std::vector<track::Cone> new_cones;
  track::Cones cameraCones;

  // Generate Fake information
  ros::Rate rate(1);

  while(ros::ok()){
    for (int i = 0; i < n_cones; ++i){
      track::Cone newCone;
      newCone.position.x = cones_x[i];
      newCone.position.y = cones_y[i];
      newCone.color = cones_col[i];

      new_cones.push_back(newCone);
    }

    cameraCones.cones = new_cones;
    send_camera.publish(cameraCones);

    ROS_INFO_STREAM("Sending random cones to camera");
    rate.sleep();
  }




  /*
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
  send_camera.publish(send_cones);
  

  return 0;
  */
}
