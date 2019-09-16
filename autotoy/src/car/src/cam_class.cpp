
#include <vector>
#include <math.h>
#include <sstream>
#include <iostream>

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
    float detect(float conex, float coney, vector<float> lst, pair <float,float> position, float acc)
    {
        float cone_loc[2] = {conex, coney};
        float ind = 0;
        float cone_found = 0;
        while(ind < acc)
        {
            float p_x = lst[ind];
            float p_y = lst[ind+1];
            float distance = abs((p_y-position.second)*conex - (p_x-position.first)*coney + p_x*position.second - p_y*position.first)/sqrt(pow((p_y-position.second), 2)+pow((p_x-position.first), 2));
            if(distance < 0.1)
            {        
                cout << "cone detected" << endl;
                return 1;
                break;
            } 
            ind = ind + 2;
        }
        cout << "cone not detected" << endl;
        return 0;
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
    float detection = dut.detect(cones[0], cones[1], view_lst, position, acc);
    cout << detection << endl;

    return 0;

}
