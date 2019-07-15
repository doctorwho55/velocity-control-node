#!/usr/bin/env python
import rospy
import sys

from duckietown_msgs.msg import Twist2DStamped

class VelocityFunction():

    def __init__(self):
        # Initiate named node
        rospy.init_node('vel_func_node', anonymous=False)

        if len(sys.argv) != 2:
            print("Vehicle name not passed as a command line argument, expected to be passed as ROS variable")
            try:
                self.hostname = rospy.get_param("~veh")
            except:
                raise Exception("ROS parameter '~veh' not found!")
        else:
            self.hostname = sys.argv[1]

        print('Hostname: %s' % self.hostname)

        # Instruct user on how to stop node
        rospy.loginfo("Use CTRL + C to stop node")

        # Function to call when ctrl + c is issued
        rospy.on_shutdown(self.shutdown)

        # Create Publisher object
        # pub = rospy.Publisher('/'+host+'/vel_func_node/car_cmd',Twist2DStamped,queue_size=1)
        self.pub = rospy.Publisher('/'+self.hostname+'/joy_mapper_node/car_cmd',Twist2DStamped,queue_size=1)

        # Create variable of message type Twist2DStamped
        self.msg = Twist2DStamped()

        # Set publish velocity rate
        self.rate = rospy.Rate(10) # 10hz

    def send_cmd(self):
        self.pub.publish(self.msg)

    def shutdown(self):
        print("Shutdown Initiated: Stopping motors")
        rospy.loginfo("Shutdown velocity function command")

        self.msg = Twist2DStamped()
        self.msg.header.stamp = rospy.Time.now()
        self.msg.v = 0.0
        self.msg.omega = 0.0
        
        self.pub.publish(self.msg)
        
        rospy.sleep(1)


if __name__ == '__main__':

    try:
        controller = VelocityFunction()
        while not rospy.is_shutdown():
            controller.send_cmd()
            controller.rate.sleep()
    except rospy.ROSInterruptException:
	    raise Exception("Error encountered when attempting to start vel_func_node velPublisher!")

