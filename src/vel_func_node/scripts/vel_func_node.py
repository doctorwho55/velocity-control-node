#!/usr/bin/env python

# Import rospy and sys
import rospy
import sys

# Import Twist2DStamped from duckietown_msgs
from duckietown_msgs.msg import Twist2DStamped


class VelocityFunction():

    def __init__(self):
        # Initialize vel_func_node node
        rospy.init_node('vel_func_node', anonymous=False)

        # Set hostname based on optional ~veh argument
        if len(sys.argv) != 2:
            rospy.loginfo(
                "Vehicle name not passed as a command line argument, expected to be passed as ROS variable")
            try:
                self.hostname = rospy.get_param("~veh")
            except:
                raise Exception("ROS parameter '~veh' not found!")
        else:
            self.hostname = sys.argv[1]
        rospy.loginfo('Hostname: %s' % self.hostname)

        # Create Publisher object
        self.pub = rospy.Publisher(
            '/'+self.hostname+'/joy_mapper_node/car_cmd', Twist2DStamped, queue_size=1)
        rospy.loginfo("Initialized publisher")

        # Create message of type duckietown/Twist2DStamped
        self.msg = Twist2DStamped()

        # Set publish rate
        self.rate = rospy.Rate(10)  # 10hz

        # Set shutdown function to call when CTRL + C is pressed
        rospy.on_shutdown(self.shutdown)

        # Instruct user on how to stop node
        rospy.loginfo("Use CTRL + C to stop node")

    def send_cmd(self):
        # Populate message to move forward
        self.msg.header.stamp = rospy.Time.now()
        self.msg.v = 0.1
        self.msg.omega = 0.0

        # Publish velocity message
        self.pub.publish(self.msg)

    def shutdown(self):
        # Log beginning of shutdown hook
        rospy.loginfo("Shutdown initiated, stopping motors")

        # Populate message to stop motion
        self.msg = Twist2DStamped()
        self.msg.header.stamp = rospy.Time.now()
        self.msg.v = 0.0
        self.msg.omega = 0.0

        # Publish motion stop message
        self.pub.publish(self.msg)
        rospy.sleep(1)

        # Log end of shutdown hook
        rospy.loginfo("Shutdown: Motors stopped")


if __name__ == '__main__':

    try:
        # Instantiate VelocityFunction object
        controller = VelocityFunction()

        while not rospy.is_shutdown():
            # Send velocity command at specified rate
            controller.send_cmd()
            controller.rate.sleep()

    except rospy.ROSInterruptException:
        raise Exception(
            "Error encountered when attempting to start vel_func_node velPublisher!")
