#!/usr/bin/env python
import rospy
from track.msg import Line
from car.msg import Control, Location

class Controller:
    
    def __init__(self):
        self.target_line_angle: float
        self.heading_angle: float
        
        # Initialise node
        rospy.init_node('controller')
        
        # Inititate publisher and subscribers
        self.publisher = rospy.Publisher('/car/controls', Control, queue_size=10)
        try:
            self.listener()
        except rospy.ROSInterruptException:
            pass
        
    def listener(self):        
        rospy.Subscriber('/car/targetline', Line, self.target_line_callback)
        rospy.Subscriber('/car/location', Location, self.location_callback)
        
        print('Listening')
        rospy.spin()
        
    def talker(self):    
        # error = self.target_line_angle - self.heading_angle
        
        # Convert error to yaw rate
        # PID stuff...
        acceleration = 4
        yaw_rate = 1
        
        self.publisher.publish(acceleration, yaw_rate)        
              
    def target_line_callback(self, data):
        # Log data to console
        rospy.loginfo(data)
    
        # Handle targetline data
        # Convert target line x,y position to angle with respect to reference coordinate system
        # self.target_line_angle = calculated_angle
        self.talker()
    
    def location_callback(self, data):
        # Log data to console
        rospy.loginfo(data)
        
        # Handle location and heading data
        # self.heading_angle = heading_angle_from_data
        self.talker()


if __name__ == '__main__':    
    c = Controller()
    