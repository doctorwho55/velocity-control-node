#!/usr/bin/env python
import rospy
import sys

from duckietown_msgs.msg import Twist2DStamped

def shutdown_hook():
    pub = rospy.Publisher('/duckiebot02/joy_mapper_node/car_cmd',Twist2DStamped,queue_size=1)
    print("Shutdown Initiated: Stopping motors.")
    msg = Twist2DStamped()
    msg.header.stamp = rospy.Time.now()
    msg.v = 0.0
    msg.omega = 0.0
    pub.publish(msg)
    rospy.sleep(1)


def velPublisher(host):
    # pub = rospy.Publisher('/'+host+'/vel_func_node/car_cmd',Twist2DStamped,queue_size=1)
    pub = rospy.Publisher('/'+host+'/joy_mapper_node/car_cmd',Twist2DStamped,queue_size=1)

    rate = rospy.Rate(10) # 10hz

    # rospy.on_shutdown(shutdown_hook)

    while not rospy.is_shutdown():
        msg = Twist2DStamped()
        msg.header.stamp = rospy.Time.now()
        msg.v = 0.1
        msg.omega = 0.0
        pub.publish(msg)
        # rospy.sleep(0.1)
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('vel_func_node', anonymous=False)
    print("Starting node %s" % rospy.get_name())

    if len(sys.argv) != 2:
        print("Vehicle name not passed as a command line argument, expected to be passed as ROS variable")
        try:
      	    hostname = rospy.get_param("~veh")
        except:
    	    raise Exception("ROS parameter '~veh' not found!")
    else:
        hostname = sys.argv[1]

    print('Hostname: %s' % hostname)

    try:
        velPublisher(host = hostname)
    except rospy.ROSInterruptException:
	    raise Exception("Error encountered when attempting to start vel_func_node velPublisher!")

