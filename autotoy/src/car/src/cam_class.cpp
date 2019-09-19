
#include "vector"
#include <math.h>
#include <sstream>

using namespace std;

// these parameters define the design of the camera of the car
int heading = 0.5; //radians
int fov = 1; //radians
int dof = 3; //meters
int acc = 300;

// this class will get the position of the car and a list of cones, and will report a list of detected cones
class bullet
{
public: //variables

    vector<float> activate(float fov, float dof, float acc, float heading, float xcar, float ycar)
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
            ang0 = ang0 - (fov/acc);
        }
        return point_lst;

    }
    bool detect(float conex, float coney, vector<float> lst, float posx, float posy, float acc)
    {
        float cone_loc[2] = {conex, coney};
        float ind = 0;
        float cone_found = 0;
        while(ind < acc)
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

// demo data
float xcar = 0;
float ycar = 0;

// make a list of cones
vector<float> cones;


int main(int argc, char **argv)
{
    // initialize the car
    bullet dut;
    vector<float> list = dut.activate(fov, dof, acc, heading, xcar, ycar);
    
    cout << list << endl;

    return 0;

}
