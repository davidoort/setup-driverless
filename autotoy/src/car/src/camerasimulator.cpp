//
// Camera Simulator Publisher 
// author: Pietro Campolucci
// funtion: the code gathers the location of the car
// and of the cones and publishes the recognized cones
//

// what it publishes: track::Cones through topic visible_cones
// what it subscribe: track::Cones from /track/cones (list of cones, only on time)
//                    car::Location from /car/location (current car position, multiple times)

// including packages
#include "ros/ros.h"
#include "track/Cones.h"
#include "car/Location.h"
#include <math.h>
#include <sstream>

using namespace std;

// these parameters define the design of the camera of the car
int fov = 1.5; //radians
int dof = 3; //meters
int acc = 100;

// this class will get the position of the car and a list of cones, and will report a list of detected cones
class bullet
{
public: //variables

    vector<float> get_orientation(float angle, pair <float,float> position)
    {
        float nose_x = position.first + 1.5*sin(angle); //add here core using angle
        float nose_y = position.second + 1.5*cos(angle);
        float tail_x = position.first - 1.5*sin(angle);
        float tail_y = position.second - 1.5*cos(angle);
        vector<float> pos;
        pos.push_back(nose_x);
        pos.push_back(nose_y);
        pos.push_back(tail_x);
        pos.push_back(tail_y);
        return pos;
    }
    vector<float> activate(float fov, float dof, float acc, car::Location position)
    {
        vector<float> point_lst;
        float ang0;
        ang0 = +(fov/2);
        while(ang0 > -fov/2)
        {
            float real = position.heading + ang0;
            float pointx = (position.location.x + dof*cos(real));
            float pointy = (position.location.y + dof*sin(real));
            point_lst.push_back(pointx);
            point_lst.push_back(pointy);
            ang0 = ang0 - (fov/acc);
        }
        return point_lst;

    }
    bool detect(track::Cone cone, vector<float> lst, car::Location position, float acc)
    {
        float cone_loc[2] = {cone.position.x, cone.position.y};
        float ind = 0;
        float cone_found = 0;
        while(ind < acc)
        {
            float p_x = lst[ind];
            float p_y = lst[ind+1];
            float distance = abs((p_y-position.location.y)*cone.position.x - (p_x-position.location.x)*cone.position.y + p_x*position.location.y - p_y*position.location.x)
            /sqrt(pow((p_y-position.location.y), 2)+pow((p_x-position.location.x), 2));
            if(distance < 0.05)
            {        
                cout << "cone detected" << endl;
                return true;
            } 
            ind = ind + 2;
        }
        cout << "cone not detected" << endl;
        return false;
    }
};

ros::Publisher camera_pub;
track::Cones cones_lst;

void carstateCallback(const car::Location& msg_car)
{
    bullet dut;
    vector<float> view_lst = dut.activate(fov, dof, acc, msg_car);
    ROS_INFO("New car position, detecting new cones...");

    track::Cones detected_cones;

    for(const track::Cone& cone : cones_lst.cones){
        bool detection = dut.detect(cone, view_lst, msg_car, acc);
        if(detection)
        {
            detected_cones.cones.push_back(cone);
        }
    }   
    camera_pub.publish(detected_cones);
}

void coneworldCallback(const track::Cones& msg_cones)
{   
    ROS_INFO("Cones arrived!");
    cones_lst = msg_cones;
}

int main(int argc, char **argv)
{
    // initialize the car

    ros::init(argc, argv, "camerasimulator");
    ros::NodeHandle n;
    camera_pub = n.advertise<track::Cones>("/car/camera", 1000); // here we define what the topic is publishing and its name
    ros::Subscriber a = n.subscribe("/car/location", 1000, carstateCallback);
    ros::Subscriber b = n.subscribe("/track/cones", 1000, coneworldCallback);
    ros::spin();
    return 0;
}
