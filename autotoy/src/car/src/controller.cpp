#include "ros/ros.h"
#include "track/Line.h"
#include "car/Control.h"
#include "car/Location.h"


class Controller {
  
  ros::NodeHandle n;
  ros::Publisher publisher;
  ros::Subscriber targetLineSub;
  ros::Subscriber locationSub;
  
  public:
    float targetLineAngle, headingAngle, velocity, velocityRef, acceleration, yawRate;
    
    // Constructor
    Controller() {      
      targetLineAngle, headingAngle, velocity, yawRate = 0;
      velocityRef = 5;
      acceleration = 4; 
      
      // Inititate publisher and subscribers
      publisher = n.advertise<car::Control>("car/controls", 1000);
      targetLineSub = n.subscribe("car/targetline", 1000, &Controller::targetLineCallback, this);
      locationSub = n.subscribe("car/location", 1000, &Controller::locationCallback, this);  
    };
    
    void talker() {
      float error = targetLineAngle - headingAngle;
      
      float Kp = 0.5;
      yawRate = Kp * error;
      
      if (velocity < velocityRef) {
        acceleration += 0.1;
        velocityRef -= 0.1;
      }
      else {
        acceleration = 0; 
      }
      
      car::Control message;
      message.acceleration = acceleration;
      message.yawrate = yawRate;            
      publisher.publish(message);
    }
    
    void targetLineCallback(const track::Line& message) {
      ROS_INFO("Received target line data");
      
      float currentPoint[2] = {message.points[0].x, message.points[0].y};
      float nextPoint[2] = {message.points[1].x, message.points[1].y};
      
      float angle = atan((nextPoint[1] - currentPoint[1]) / (nextPoint[0] - currentPoint[0]));
      if (angle < 0) {
        angle += 180;
      }
      
      targetLineAngle = angle;
      
      talker();
    }
    
    void locationCallback(const car::Location& message) {
      ROS_INFO("Received location data");
      
      headingAngle = message.heading;
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