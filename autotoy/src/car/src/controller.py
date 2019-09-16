#!/usr/bin/env python
import rospy
import math
from track.msg import Line
from car.msg import Control, Location

class Controller:
    
    def __init__(self):
        self.target_line_angle = 0
        self.heading_angle = 0
        
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
        error = self.target_line_angle - self.heading_angle
        
        # Convert error to yaw rate
        # PID stuff...
        acceleration = 4
        yaw_rate = 1
        
        self.publisher.publish(acceleration, yaw_rate)        
              
    def target_line_callback(self, data):
        print('Received target line data')
        
        current_point = (data.points[0].x, data.points[0].y)
        next_point = (data.points[1].x, data.points[1].y)
        
        angle = math.atan((next_point[1] - current_point[1]) / (next_point[0] - current_point[0]))
        if angle < 0: 
            angle += 180
             
        self.target_line_angle = angle
    
        self.talker()
    
    def location_callback(self, data):
        print('Received location and heading')
        
        # Handle location and heading data
        self.heading_angle = data.heading


if __name__ == '__main__':    
    Controller()
    