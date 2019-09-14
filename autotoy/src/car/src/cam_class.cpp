#include <math.h>
#include <sstream>
#include <iostream>

class car
{
public: //variables
    std::pair <int,int> position;
    float nose_x;
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
        float pos[4];
        pos[0] = nose_x;
        pos[1] = nose_y;
        pos[2] = tail_x;
        pos[3] = tail_y;
        std::cout << pos[0] << std::endl;
        std::cout << 1.5*sin(angle*M_PI/180) << std::endl;
    }
    void project(int fov, int angle, int dof, int acc)
    {

    }
};

int angle_1 = 20; //degrees
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
    position = std::make_pair (60, 50);
    dut.get_orientation(angle_1, position);
    dut.project(fov, angle_1, dof, acc);
    return 0;

}
