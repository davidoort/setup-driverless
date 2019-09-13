//
// Camera Simulator Publisher 
// author: Pietro Campolucci
// funtion: the code gathers the location of the car
// and of the cones and publishes the recognized cones
//

#include "ros/ros.h"
#include "std_msgs/String.h"

# include <sstream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "camerasimulator");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000); // here we specify the nature of the topic and its name
    ros::Rate loop_rate(10);

    int count = 0;
    while (ros::ok())
    {
        std_msgs::String msg;

        std::stringstream ss;
        ss << "cone location list"; // <<count
        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str());
        chatter_pub.publish(msg); // actual broadcast message of the datum 
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}
