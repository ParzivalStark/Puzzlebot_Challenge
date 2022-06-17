#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def set_vel(pub, velocity):
    msg = Twist()
    msg.linear.x = velocity[0][0]
    msg.linear.y = velocity[0][1]
    msg.linear.z = velocity[0][2]
    msg.angular.x = velocity[1][0]
    msg.angular.y = velocity[1][1]
    msg.angular.z = velocity[1][2]
    pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('linea')
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    rospy.loginfo("Eeeey tamos de vuelta")
    rate = rospy.Rate(10)
    msg = Twist()
    while not rospy.is_shutdown():
        msg.linear.x = 1
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = 0
        pub.publish(msg)
        #set_vel(pub, [[0.5, 0, 0], [0, 0, 0]])
        rospy.sleep(3)
        msg.linear.x = 0
        pub.publish(msg)
        #set_vel(pub, [[0.0, 0, 0], [0, 0, 0]])
        #rospy.signal_shutdown("Finished")
        rate.sleep()

