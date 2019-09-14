//
// Camera Simulator Publisher 
// author: Pietro Campolucci
// funtion: the code gathers the location of the car
// and of the cones and publishes the recognized cones
//

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include "track/Point.h"
#include <math.h>
#include <sstream>

class car
{
public: //variables
    std::pair <std_msgs::Float32,std_msgs::Float32> position;
    float nose_x;
    float nose_y;
    float tail_x;
    float tail_y;
    int cones_seen;

    float get_orientation(float angle, std::pair <float,float> position)
    {
        nose_x = position.first + 1.5*sin(angle*M_PI/180); //add here core using angle
        nose_y = position.second + 1.5*cos(angle*M_PI/180);
        tail_x = position.first + 1.5*sin(angle*M_PI/180);
        tail_y = position.second + 1.5*cos(angle*M_PI/180);
        float pos[4];
        pos[0] = nose_x;
        pos[1] = nose_y;
        pos[2] = tail_x;
        pos[3] = tail_y;
        return pos[0], pos[1];
    }
    void project(int fov, int angle, int dof, int acc)
    {

    }
};

float angle = 20.0; //degrees
int fov = 120; //degrees
int dof = 30;
int acc = 100;

int main(int argc, char **argv)
{
    // what i want to do with the upper class?
    // get the orientation of the car
    // get the list of cones
    car dut;
    std::pair <float,float> position;
    position = std::make_pair (50.0, 50.0);
    dut.get_orientation(angle, position);
    dut.project(fov, angle, dof, acc);

    ros::init(argc, argv, "camerasimulator");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<track::Point>("chatter", 1000); // here we specify the nature of the topic and its name
    ros::Rate loop_rate(10);

    int count = 0;
    while (ros::ok())
    {
        track::Point msg;
        msg.x, msg.y = dut.get_orientation(angle, position);
        ROS_INFO_STREAM(" x =" << msg.x << " y = " << msg.y);
        chatter_pub.publish(msg); // actual broadcast message of the datum 
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}
