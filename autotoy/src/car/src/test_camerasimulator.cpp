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
float dof = 50; //meters

// the class will provide functions to plot the fov and use it to detect the cones in the way
class car
{
public:

    // plotting field of view as an arc of points (returns a list of points)
    vector<float> activate(float fov, float angle, float dof, pair <float,float> position)
    {
        vector<float> point_lst;
        float ang1 = -fov/2 + angle;
        float ang2 = fov/2 + angle;
        float pointx1 = (position.first + dof*cos(ang1));
        float pointy1 = (position.second + dof*sin(ang1));
        point_lst.push_back(pointx1);
        point_lst.push_back(pointy1);
        float pointx2 = (position.first + dof*cos(ang2));
        float pointy2 = (position.second + dof*sin(ang2));
        point_lst.push_back(pointx2);
        point_lst.push_back(pointy2);
        return point_lst;
    }

    // gets the coord of one cone and returns if it is detected or not (returns 1 for "yes" and 0 for "no")
    // NOTE: the print out messages will be reported once the node receives cones positions
    float detect(float conex, float coney, vector<float> lst, pair <float,float> position)
    {       
        float Area = (position.first*(lst[1]-lst[3]) + lst[0]*(lst[3]-position.second) + lst[2]*(position.second-lst[1]))/2;
        float sub_area_1 = abs((position.first*(coney-lst[3]) + conex*(lst[3]-position.second) + lst[2]*(position.second-coney))/2);
        float sub_area_2 = abs((position.first*(coney-lst[1]) + conex*(lst[1]-position.second) + lst[0]*(position.second-coney))/2);
        float sub_area_3 = abs((lst[0]*(coney-lst[3]) + conex*(lst[3]-lst[1]) + lst[2]*(lst[1]-coney))/2);

        // debugging
        // cout << Area << endl;
        // cout << sub_area_1+sub_area_2+sub_area_3 << endl;
        // cout << Area-sub_area_1-sub_area_2-sub_area_3 << endl;

        if(abs(Area-sub_area_1-sub_area_2-sub_area_3) < 0.1){
            cout << "cone detected" << endl;
            return 1;
        } else {
            cout << "cone not detected" << endl;
            return 0;
        }
    }
};

// DEMO DATA: not for the final code, just to test if the code works
float cones[3] = {30, 60, 1}; // posx, posy and color of the cone
pair <float,float> position = make_pair (50, 50);
float orientation = M_PI/2; // radians, respect to the y axis, clockwise


int main(int argc, char **argv)
{
    // initialize the car
    car dut;

    // get fov and use it to get the detection statement
    vector<float> view_lst = dut.activate(fov, orientation, dof, position);
    float detection = dut.detect(cones[0], cones[1], view_lst, position);

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
    file_1 << position.first << ' ' << position.second <<std::endl;
    file_1 << view_lst[0] << ' ' << view_lst[1] <<std::endl;
    file_1.close();
    return 0;
}
