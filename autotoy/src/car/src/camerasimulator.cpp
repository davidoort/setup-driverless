//
// Camera Simulator Publisher 
// author: Pietro Campolucci
// funtion: the code gathers the location of the car
// and of the cones and publishes the recognized cones
//

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32MultiArray.h"
#include <math.h>
#include <sstream>

class car
{
public: //variables
    std::pair <int,int> position;
    int nose_x;
    int nose_y;
    int tail_x;
    int tail_y;
    int cones_seen;

    void get_orientation(int angle, std::pair <int,int> position)
    {
        nose_x = position.first + 1.5*sin(angle*M_PI/180); //add here core using angle
        nose_y = position.second + 1.5*cos(angle*M_PI/180);
        tail_x = position.first + 1.5*sin(angle*M_PI/180);
        tail_y = position.second + 1.5*cos(angle*M_PI/180);
        int pos[4];
        pos[0] = nose_x;
        pos[1] = nose_y;
        pos[2] = tail_x;
        pos[3] = tail_y;
        std::cout << pos[4] << std::endl;
    }
    void project(int fov, int angle, int dof, int acc)
    {

    }
};

int angle = 20; //degrees
int fov = 120; //degrees
int dof = 30;
int acc = 100;

int main(int argc, char **argv)
{
    // what i want to do with the upper class?
    // get the orientation of the car
    // get the list of cones
    car dut;
    std::pair <int,int> position;
    position = std::make_pair (50, 50);
    dut.get_orientation(angle, position);
    dut.project(fov, angle, dof, acc);

    ros::init(argc, argv, "camerasimulator");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::Int8>("chatter", 1000); // here we specify the nature of the topic and its name
    ros::Rate loop_rate(10);

    int count = 0;
    while (ros::ok())
    {
        std_msgs::Int8 msg;
        msg.data = 2;
        ROS_INFO("%d", msg.data);
        chatter_pub.publish(msg); // actual broadcast message of the datum 
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}
