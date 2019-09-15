import rospy
from std_msgs import String

class Controller:

    def __init__(self):
        self.target_line_angle: float
        self.heading_angle: float

        # Inititate publisher and subscribers
        self.listener()
        self.talker()

    def listener(self):
        rospy.init_node('controller')

        rospy.Subscriber('/car/targetline', String, self.target_line_callback) # 'String' should be replace by 'Line' message type when it is created by the track finder guys
        rospy.Subscriber('/car/location', String, self.location_callback)   # 'String' should be replace by 'Location' message type when it is created by the location guys 

        rospy.spin()

    def talker(self):    
        error = self.target_line_angle - self.heading_angle

        # PI controller for error
        PI on error
        # New yaw rate in degrees
        yaw_angle += error

        # new acceleration in ms^-2
        # error_acc = doulbe integrate (/car/location - expected_/car/location)
        # acceleration = current acc + error_acc

        publisher = rospy.Publisher('/car/controls', String)
        rospy.init_node('talker', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            rospy.loginfo('')
            publisher.publish('')
            rate.sleep()

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
    c = Controller()  # why c = 
