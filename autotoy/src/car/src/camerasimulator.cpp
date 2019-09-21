#include "ros/ros.h"
#include "track/Cones.h"
#include "car/Location.h"
#include <math.h>
#include <sstream>

using namespace std;

// =============================================================================================
// Camera Simulator Publisher 
// author: Pietro Campolucci
// funtion: the code gathers the location of the car
// and of the cones and publishes the recognized cones
// 
// what it publishes: track::Cones through topic visible_cones
// what it subscribe: track::Cones from /track/cones (list of cones, only on time)
//                    car::Location from /car/location (current car position, multiple times)
// =============================================================================================

// these parameters define the design of the camera of the car
float fov = M_PI/2; //radians
float dof = 30; //meters
float acc = 200; // NOTE: if you decrease the accuracy, you need to increase the max distance for detection

// this class will get the position of the car and a list of cones, and will report a list of detected cones
class bullet
{
public:

    // plotting field of view as an arc of points (returns a list of points)
    vector<float> activate(float fov, float dof, float acc, car::Location position)
    {
        vector<float> point_lst;
        float ang1 = -fov/2 + position.heading;
        float ang2 = fov/2 + position.heading;
        float pointx1 = (position.location.x + dof*cos(ang1));
        float pointy1 = (position.location.y + dof*sin(ang1));
        point_lst.push_back(pointx1);
        point_lst.push_back(pointy1);
        float pointx2 = (position.location.x + dof*cos(ang2));
        float pointy2 = (position.location.y + dof*sin(ang2));
        point_lst.push_back(pointx2);
        point_lst.push_back(pointy2);
        return point_lst;
    }

    // gets the coord of one cone and returns if it is detected or not (returns 1 for "yes" and 0 for "no")
    // NOTE: the print out messages will be reported once the node receives cones positions
    bool detect(track::Cone cone, vector<float> lst, car::Location position, float acc)
    {
        float Area = (position.location.x*(lst[1]-lst[3]) + lst[0]*(lst[3]-position.location.y) + lst[2]*(position.location.y-lst[1]))/2;
        float sub_area_1 = abs((position.location.x*(cone.position.y-lst[3]) + cone.position.x*(lst[3]-position.location.y) + lst[2]*(position.location.y-cone.position.y))/2);
        float sub_area_2 = abs((position.location.x*(cone.position.y-lst[1]) + cone.position.x*(lst[1]-position.location.y) + lst[0]*(position.location.y-cone.position.y))/2);
        float sub_area_3 = abs((lst[0]*(cone.position.y-lst[3]) + cone.position.x*(lst[3]-lst[1]) + lst[2]*(lst[1]-cone.position.y))/2);

        if(abs(Area-sub_area_1-sub_area_2-sub_area_3) < 0.1){
            cout << "cone detected" << endl;
            return 1;
        } else {
            cout << "cone not detected" << endl;
            return 0;
        }
    }
};

ros::Publisher camera_pub;
track::Cones cones_lst;

// get the car location and publish the list of new cones
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
    ROS_INFO("Sending position of all the detected cones");  
    camera_pub.publish(detected_cones);
}

// get cones location just one time for further use
void coneworldCallback(const track::Cones& msg_cones)
{   
    ROS_INFO("Cones arrived!");
    cones_lst = msg_cones;
}

int main(int argc, char **argv)
{
    // initialize ros
    ros::init(argc, argv, "camerasimulator");
    ros::NodeHandle n;

    // publish cones location by car loaction info and cones location
    camera_pub = n.advertise<track::Cones>("/car/camera", 1000); 
    ros::Subscriber a = n.subscribe("/car/location", 1000, carstateCallback);
    ros::Subscriber b = n.subscribe("/track/cones", 1000, coneworldCallback);
    ros::spin();
    return 0;
}