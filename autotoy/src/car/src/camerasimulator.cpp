//
// Camera Simulator Publisher 
// author: Pietro Campolucci
// funtion: the code gathers the location of the car
// and of the cones and publishes the recognized cones
//

// including packages
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "track/Cones.h"
#include "track/Cone.h"
#include "track/Line.h"
#include <math.h>
#include <sstream>

using namespace std;

// these parameters define the design of the camera of the car
int fov = 120; //degrees
int dof = 30;
int acc = 100;

// this class will get the position of the car and a list of cones, and will report a list of detected cones
class car
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
    vector<float> activate(float fov, float angle, float dof, float acc, pair <float,float> position)
    {
        vector<float> point_lst;
        float ang0;
        ang0 = -(fov/2);
        while(ang0 < fov/2)
        {
            float real = angle + ang0;
            float pointx = (position.first + dof*sin(real));
            float pointy = (position.second + dof*cos(real));
            point_lst.push_back(pointx);
            point_lst.push_back(pointy);
            ang0 = ang0 + (fov/acc);
        }
        return point_lst;

    }
    vector<float> detect(float conex, float coney, vector<float> lst, pair <float,float> position, float acc, float color)
    {
        float cone_loc[2] = {conex, coney};
        float ind = 0;
        float cone_found = 0;
        while(ind < acc)
        {
            float p_x = lst[ind];
            float p_y = lst[ind+1];
            float distance = abs((p_y-position.second)*conex - (p_x-position.first)*coney + p_x*position.second - p_y*position.first)
            /sqrt(pow((p_y-position.second), 2)+pow((p_x-position.first), 2));
            if(distance < 0.1)
            {        
                cout << "cone detected" << endl;
                vector<float> cone_location;
                cone_location.push_back(conex);
                cone_location.push_back(coney);
                cone_location.push_back(color); // cone color, to be implemented
                return cone_location;
                break;
            } 
            ind = ind + 2;
        }
        cout << "cone not detected" << endl;
        vector<float> nothing;
        return nothing;
    }
};

// DEMO DATA: not for the final code, just to test if the code works
float cones[3] = {50, 50, 1}; // posx, posy and color of the cone
pair <float,float> position = make_pair (50, 50);
float orientation = 0.5; // radians, respect to the y axis


int main(int argc, char **argv)
{
    // initialize the car
    car dut;

    vector<float> view_lst = dut.activate(fov, orientation, dof, acc, position);
    vector<float> detection = dut.detect(cones[0], cones[1], view_lst, position, acc, cones[2]);

    ros::init(argc, argv, "camerasimulator");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<track::Cones>("chatter", 1000); // here we specify the nature of the topic and its name
    ros::Rate loop_rate(10);

    int count = 0;
    while (ros::ok())
    {
        track::Cone cone;
        cone.position.x = detection[0];
        cone.position.y = detection[1];
        cone.color = detection[2];

        track::Cones detected_cones;
        detected_cones.cones.push_back(cone);
        
        chatter_pub.publish(detected_cones);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}
