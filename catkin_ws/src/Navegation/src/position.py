#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from math import *

class botPosition:

    def __init__(self, r, l):
        rospy.init_node("botPosition")
    
        self.pub = rospy.Publisher("/position", Twist, queue_size=10)
        rospy.Subscriber("/wr", Float32, self.wr_callback)
        rospy.Subscriber("/wl", Float32, self.wl_callback)

        self.rate = rospy.Rate(10)
        self.r = r
        self.l = l
        self.wr = 0
        self.wl = 0

        self.pos = Twist()
        self.pos.linear.x = 0
        self.pos.linear.y = 0
        self.pos.linear.z = 0
        self.pos.angular.x = 0
        self.pos.angular.y = 0
        self.pos.angular.z = 0
        self.last_t = rospy.get_time()
    
    def wr_callback(self, velocity):
        self.wr = velocity.data

    def wl_callback(self, velocity):
        self.wl = velocity.data

    def main(self):
        while not rospy.is_shutdown():
            current_t = rospy.get_time()
            dt = current_t - self.last_t
            self.last_t = current_t
            
            distance = self.r * (self.wr + self.wl) * 0.5 * dt
            self.pos.angular.z += self.r * (self.wr - self.wl) / self.l * dt

            if self.pos.angular.z < -pi:
                self.pos.angular.z += 2*pi
            elif self.pos.angular.z > pi:
                self.pos.angular.z -= 2*pi

            xd = distance*cos(self.pos.angular.z)
            yd = distance*sin(self.pos.angular.z)

            self.pos.linear.x += xd
            self.pos.linear.y += yd

            self.pub.publish(self.pos)

            self.rate.sleep()


if __name__ == '__main__':
    try:
        node = botPosition(0.05, 0.19)
        node.main()

    except rospy.ROSInterruptException:
        pass