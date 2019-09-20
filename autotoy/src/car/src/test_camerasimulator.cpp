#include <vector>
#include <math.h>
#include <sstream>
#include <iostream>
#include <fstream>

using namespace std;

// =============================================================
// Test Code for Camera Simulation
// Description: the code will send the cone coordinates to
// cone_loc.txt and the fov coordinates to fov_loc.txt, for 
// further car visualization
// =============================================================

// these parameters define the design of the camera of the car
float fov = M_PI/2; //radians
float dof = 30; //meters
float acc = 200; //number of lines

// the class will provide functions to plot the fov and use it to detect the cones in the way
class car
{
public:

    // plotting field of view as an arc of points (returns a list of points)
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
            ang0 = ang0 + fov/acc;
        }
        return point_lst;
    }

    // gets the coord of one cone and returns if it is detected or not (returns 1 for "yes" and 0 for "no")
    // NOTE: the print out messages will be reported once the node receives cones positions
    float detect(float conex, float coney, vector<float> lst, pair <float,float> position, float acc)
    {
        float cone_loc[2] = {conex, coney};
        float ind = 0;
        float cone_found = 0;
        while(ind < lst.size())
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
float cones[3] = {30, 60, 1}; // posx, posy and color of the cone
pair <float,float> position = make_pair (50, 50);
float orientation = M_PI*1.43; // radians, respect to the y axis, clockwise


int main(int argc, char **argv)
{
    // initialize the car
    car dut;

    // get fov and use it to get the detection statement
    vector<float> view_lst = dut.activate(fov, orientation, dof, acc, position);
    float detection = dut.detect(cones[0], cones[1], view_lst, position, acc);

    // store information of the point to cone_loc.txt
    std::ofstream file;
    file.open("cone_loc.txt");
    file << 'x' << ' ' << 'y' << '\n';
    file << cones[0] << ' ' << cones[1] <<std::endl;
    file.close();

    // store information of the camera field to fov_loc.txt
    std::ofstream file_1;
    file_1.open("fov_loc.txt");
    int ind = 0;
    file_1 << 'x' << ' ' << 'y' << '\n';
    while(ind<(view_lst.size())){
        file_1 << view_lst[ind] << ' ' << view_lst[ind+1] <<std::endl; 
        ind = ind + 2;
    }
    file_1.close();
    return 0;
}
