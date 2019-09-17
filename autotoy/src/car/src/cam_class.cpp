
#include "ros/ros.h"
#include "track/Cones.h"
#include "car/Location.h"
#include <math.h>
#include <sstream>

using namespace std;

// these parameters define the design of the camera of the car
int fov = 120; //degrees
int dof = 30;
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
            if(distance < 0.1)
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

// DEMO DATA: not for the final code, just to test if the code works



int main(int argc, char **argv)
{
    // initialize the car
    bullet dut;

    vector<float> view_lst = dut.activate(fov, orientation, dof, acc, position);
    float detection = dut.detect(cones[0], cones[1], view_lst, position, acc);
    cout << detection << endl;

    return 0;

}
