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
        error = self.target_line_angle - self.heading_angle
        
        # Convert error to yaw rate
        yaw_rate = 1
        
        # PID stuff... pip install simple-pid
        from simple_pid import PID
        pid = PID(1, 0.1, 0.05, setpoint=1)

        # assume we have a system we want to control in controlled_system
        v = controlled_system.update(0)

        while True:
        # compute new ouput from the PID according to the systems current value
        control = pid(v)
        # feed the PID output to the system and get its current value
        v = controlled_system.update(control)

        pid.sample_time = 0.01  # update every 0.01 seconds

        while True:
            output_yaw = pid(v)


        acceleration = 4
        if straighline:
            acceleration =+ .1
        else:
            acceleration = 0
        
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
    