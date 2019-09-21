<<<<<<< HEAD:autotoy/src/car/src/test_camerasimulator.cpp
#include <vector>
#include <math.h>
#include <sstream>
#include <iostream>
#include <fstream>
=======

#include "vector"
#include <math.h>
#include <sstream>
>>>>>>> Camera_Simulation:autotoy/src/car/src/cam_class.cpp

using namespace std;

// =============================================================
// Test Code for Camera Simulation
// Description: the code will send the cone coordinates to
// cone_loc.txt and the fov coordinates to fov_loc.txt, for 
// further car visualization
// =============================================================

// these parameters define the design of the camera of the car
<<<<<<< HEAD:autotoy/src/car/src/test_camerasimulator.cpp
float fov = M_PI/2; //radians
float dof = 30; //meters
float acc = 200; //number of lines

// the class will provide functions to plot the fov and use it to detect the cones in the way
class car
=======
int heading = 0.5; //radians
int fov = 1; //radians
int dof = 3; //meters
int acc = 300;

// this class will get the position of the car and a list of cones, and will report a list of detected cones
class bullet
>>>>>>> Camera_Simulation:autotoy/src/car/src/cam_class.cpp
{
public:

<<<<<<< HEAD:autotoy/src/car/src/test_camerasimulator.cpp
    // plotting field of view as an arc of points (returns a list of points)
    vector<float> activate(float fov, float angle, float dof, float acc, pair <float,float> position)
=======
    vector<float> activate(float fov, float dof, float acc, float heading, float xcar, float ycar)
>>>>>>> Camera_Simulation:autotoy/src/car/src/cam_class.cpp
    {
        vector<float> point_lst;
        float ang0;
        ang0 = +(fov/2);
        while(ang0 > -fov/2)
        {
            float real = heading + ang0;
            float pointx = (xcar + dof*cos(real));
            float pointy = (ycar + dof*sin(real));
            point_lst.push_back(pointx);
            point_lst.push_back(pointy);
<<<<<<< HEAD:autotoy/src/car/src/test_camerasimulator.cpp
            ang0 = ang0 + fov/acc;
=======
            ang0 = ang0 - (fov/acc);
>>>>>>> Camera_Simulation:autotoy/src/car/src/cam_class.cpp
        }
        return point_lst;
    }
<<<<<<< HEAD:autotoy/src/car/src/test_camerasimulator.cpp

    // gets the coord of one cone and returns if it is detected or not (returns 1 for "yes" and 0 for "no")
    // NOTE: the print out messages will be reported once the node receives cones positions
    float detect(float conex, float coney, vector<float> lst, pair <float,float> position, float acc)
=======
    bool detect(float conex, float coney, vector<float> lst, float posx, float posy, float acc)
>>>>>>> Camera_Simulation:autotoy/src/car/src/cam_class.cpp
    {
        float cone_loc[2] = {conex, coney};
        float ind = 0;
        float cone_found = 0;
        while(ind < lst.size())
        {
            float p_x = lst[ind];
            float p_y = lst[ind+1];
            float distance = abs((p_y-posy)*conex - (p_x-posx)*coney + p_x*posy - p_y*posx)
            /sqrt(pow((p_y-posy), 2)+pow((p_x-posx), 2));
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

<<<<<<< HEAD:autotoy/src/car/src/test_camerasimulator.cpp
// DEMO DATA: not for the final code, just to test if the code works
float cones[3] = {30, 60, 1}; // posx, posy and color of the cone
pair <float,float> position = make_pair (50, 50);
float orientation = M_PI*1.43; // radians, respect to the y axis, clockwise
=======
// demo data
float xcar = 0;
float ycar = 0;

// make a list of cones
vector<float> cones;
>>>>>>> Camera_Simulation:autotoy/src/car/src/cam_class.cpp


int main(int argc, char **argv)
{
    // initialize the car
<<<<<<< HEAD:autotoy/src/car/src/test_camerasimulator.cpp
    car dut;

    // get fov and use it to get the detection statement
    vector<float> view_lst = dut.activate(fov, orientation, dof, acc, position);
    float detection = dut.detect(cones[0], cones[1], view_lst, position, acc);
=======
    bullet dut;
    vector<float> list = dut.activate(fov, dof, acc, heading, xcar, ycar);
    
    cout << list << endl;
>>>>>>> Camera_Simulation:autotoy/src/car/src/cam_class.cpp

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
