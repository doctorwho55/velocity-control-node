#!/usr/bin/env python
import rospy
from duckietown_msgs.msg import Twist2DStamped

# Initialize the node with rospy
rospy.init_node('publisher_node', anonymous=False)

# Create publisher
publisher = rospy.Publisher("commanded_velocity",Twist2DStamped,queue_size=1)

# Publish every 1 second
while not rospy.is_shutdown():
    msg = Twist2DStamped()
    msg.header.stamp = rospy.Time.now()
    msg.v = 0.1
    msg.omega = 0.0
    publisher.publish(msg)
    rospy.sleep(0.1)
rospy.spin() #Keeps the script for exiting
