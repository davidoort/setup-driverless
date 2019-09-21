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
<<<<<<< HEAD
float fov = M_PI/2; //radians
float dof = 20; //meters
float acc = 200; // NOTE: if you decrease the accuracy, you need to increase the max distance for detection
=======
int fov = 1.5; //radians
int dof = 3; //meters
int acc = 100;
>>>>>>> Camera_Simulation

// this class will get the position of the car and a list of cones, and will report a list of detected cones
class bullet
{
public:

    // plotting field of view as an arc of points (returns a list of points)
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
<<<<<<< HEAD
            ang0 = ang0 + fov/acc;
=======
            ang0 = ang0 - (fov/acc);
>>>>>>> Camera_Simulation
        }
        return point_lst;
    }

    // gets the coord of one cone and returns if it is detected or not (returns 1 for "yes" and 0 for "no")
    // NOTE: the print out messages will be reported once the node receives cones positions
    bool detect(track::Cone cone, vector<float> lst, car::Location position, float acc)
    {
        float cone_loc[2] = {cone.position.x, cone.position.y};
        float ind = 0;
        float cone_found = 0;
        while(ind < lst.size())
        {
            float p_x = lst[ind];
            float p_y = lst[ind+1];
            float distance = abs((p_y-position.location.y)*cone.position.x - (p_x-position.location.x)*cone.position.y + p_x*position.location.y - p_y*position.location.x)/sqrt(pow((p_y-position.location.y), 2)+pow((p_x-position.location.x), 2));
            if(distance < 0.1)
            {        
                cout << "cone detected" << endl;
                return true;
                break;
            } 
            ind = ind + 2;
        }
        cout << "cone not detected" << endl;
        return false;
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
    ROS_INFO("SEnding position of all the detected cones");  
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
