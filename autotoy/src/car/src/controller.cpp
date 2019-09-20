#include "ros/ros.h"
#include "track/Line.h"
#include "car/Control.h"
#include "car/Location.h"
#include "car/Velocity.h"
#include <vector>

using namespace std;


class Controller {
  
  ros::NodeHandle nodeHandler;
  ros::Publisher publisher;
  ros::Subscriber targetLineSub;
  ros::Subscriber locationSub;
  ros::Subscriber velocitySub;
  
  public:
    float targetLineAngle, headingAngle, velocity, velocityRef, acceleration, yawRate;
    
    // Constructor
    Controller() {      
      targetLineAngle, headingAngle, velocity, yawRate, acceleration = 0;
      velocityRef = 3; // m/s
      
      // Inititate publisher and subscribers
      publisher = nodeHandler.advertise<car::Control>("car/controls", 1000);
      targetLineSub = nodeHandler.subscribe("car/targetline", 1000, &Controller::targetLineCallback, this);
      locationSub = nodeHandler.subscribe("car/location", 1000, &Controller::locationCallback, this); 
      velocitySub = nodeHandler.subscribe("car/velocity", 1000, &Controller::velocityCallback, this);
    };
    
    void talker() {
      float headingError = targetLineAngle - headingAngle;
      float velocityError = velocityRef - velocity;
      
      // Proportional control
      float Kp_yaw = 1.;
      float Kp_a = 0.5;
      yawRate = Kp_yaw * headingError;
      acceleration = Kp_a * velocityError;
            
      car::Control message;
      message.acceleration = acceleration;
      message.yawrate = yawRate;            
      publisher.publish(message);
    }
    
    void targetLineCallback(const track::Line& message) {
      ROS_INFO("Received target line data");
      
      if (message.points.size() > 0) {
        vector<float> currentPoint = {message.points[0].x, message.points[0].y};
        vector<float> nextPoint = {message.points[1].x, message.points[1].y};
        
        float angle = atan((nextPoint[1] - currentPoint[1]) / (nextPoint[0] - currentPoint[0] + 0.001));
        if (angle < 0) {
          angle += 180;
        }
        ROS_ERROR("%f", nextPoint[1]);
        
        targetLineAngle = angle;
      }
      
      talker();
    }
    
    void locationCallback(const car::Location& message) {
      ROS_INFO("Received location data");
      
      headingAngle = message.heading;
    }  
    
    void velocityCallback(const car::Velocity& message) {
      ROS_INFO("Received current velocity");
      
      velocity = message.velocity;
    }
  
};


int main(int argc, char **argv) { 
  
  // Initiate ros node
  ros::init(argc, argv, "controller");
  
  // Instantiate Controller class
  Controller controller;
  
  ros::spin();
  
  return 0;
}