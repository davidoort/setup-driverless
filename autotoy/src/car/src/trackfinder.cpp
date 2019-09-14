#include "ros/ros.h"
#include "track/Point.h"

#include <sstream>

int main(int argc, char **argv) {
  // Init node with name 'trackfinder'
  ros::init(argc, argv, "trackfinder");

  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<track::Point>("/car/targetline", 1000);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    track::Point msg;

    std::stringstream ss;
    ss << "Here is line " << count << "!";
    msg.x = 2.3;
    msg.y = 0.8;

    // Log the message
    //ROS_INFO("%s", msg.data.c_str());

    // Send the message
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
