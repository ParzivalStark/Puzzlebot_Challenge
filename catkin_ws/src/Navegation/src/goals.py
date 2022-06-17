#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from math import sqrt

class Goals:
    def __init__(self):
        rospy.init_node('Goals')
        self.pub = rospy.Publisher("/goal", Twist, queue_size = 10)
        rospy.Subscriber("/position", Twist, self.pos_callback)

        self.x = 0
        self.y = 0

        self.i = 0

        self.rate = rospy.Rate(10)

        self.next = Twist()
        self.next.linear.x = 0
        self.next.linear.y = 0
        self.next.linear.z = 0
        self.next.angular.x = 0
        self.next.angular.y = 0
        self.next.angular.z = 0

        self.goals = [[0, 0],[0.4, 0],[0.8, 0], [ 1.0, 0.2], [1.2, 0.4], [1.0, 0.6], [0.8, 0.8], [0.6, 0.6], [0.4, 0.4], [0.4, 0]]

    def pos_callback(self, pos):
        self.x = pos.linear.x
        self.y = pos.linear.y

    def finished():
        rospy.loginfo("Done")

    def main(self):
        rospy.sleep(0.1)
        while not rospy.is_shutdown():
            if self.i < len(self.goals) - 1:
                if sqrt((self.x - self.goals[self.i][0])**2 + (self.y - self.goals[self.i][1])**2) <= 0.01:
                    rospy.loginfo('smn')
                    self.next.linear.x = self.goals[self.i+1][0]
                    self.next.linear.y = self.goals[self.i+1][1]
                    rospy.loginfo(self.next)
                    self.pub.publish(self.next)
                    self.i += 1
                self.rate.sleep()
            else:
                rospy.signal_shutdown(self.finished)

if __name__ == '__main__':
    goals = Goals()
    goals.main()