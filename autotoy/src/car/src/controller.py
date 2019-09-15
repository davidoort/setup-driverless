import rospy
from std_msgs import String
from track.msg import Line
from car.msg import Control, Location

class Controller:
    
    def __init__(self):
        self.target_line_angle: float
        self.heading_angle: float
        
        # Inititate publisher and subscribers
        self.listener()
        try:
            self.talker()
        except rospy.ROSInterruptException:
            pass
        
    def listener(self):
        rospy.init_node('controller')
        
        rospy.Subscriber('/car/targetline', Line, self.target_line_callback)
        rospy.Subscriber('/car/location', Location, self.location_callback)
        
        rospy.spin()
        
    def talker(self):    
        error = self.target_line_angle - self.heading_angle
        
        # Convert error to yaw rate
        # PID stuff...
        
        publisher = rospy.Publisher('/car/controls', String)
        rospy.init_node('')
        
              
    def target_line_callback(self, data):
        # Log data to console
        rospy.loginfo(data.data)
    
        # Handle targetline data
        # Convert target line x,y position to angle with respect to reference coordinate system
        self.target_line_angle = calculated_angle
    
    def location_callback(self, data):
        # Log data to console
        rospy.loginfo(data.data)
        
        # Handle location and heading data
        self.heading_angle = heading_angle_from_data


if __name__ == '__main__':    
    c = Controller()
    