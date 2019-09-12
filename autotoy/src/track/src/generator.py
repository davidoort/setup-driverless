#!usr/bin/env python
import rospy
from track/srv import Generator

def add(req):
    return 

def server():
    rospy.init_node("generator_node")
    s = rospy.Service("track/generate", Generator , add)
    print("Ready to send points ")
    rospy.spin()

if __name__ == "__main__":
    server()



