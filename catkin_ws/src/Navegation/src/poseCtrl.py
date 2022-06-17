#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from math import *
import rosbag

class trajectoryFollowing:
    def __init__(self, kp, ka, kb):
        rospy.init_node("trajectory_Following")
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/position", Twist, self.pos_callback)

        self.rate = rospy.Rate(10)

        self.current_goal = 0
        self.x = 0
        self.y = 0
        self.theta = 0
        self.kp = kp
        self.ka = ka
        self.kb = kb
        self.xd = [1.2, 1.2, 0]
        self.yd = [-0.6, 1.2, 0]
        self.thetaRef = [-pi/2, 0, pi/2]
        self.de = 0.01

        self.vel = Twist()
        self.vel.linear.x = 0
        self.vel.linear.y = 0
        self.vel.linear.z = 0
        self.vel.angular.x = 0
        self.vel.angular.y = 0
        self.vel.angular.z = 0

        self.bag = rosbag.Bag('error.bag', 'w')

    def pos_callback(self, pos):
        self.x = pos.linear.x
        self.y = pos.linear.y
        self.theta = pos.angular.z

    def stop(self):
        self.vel.linear.x = 0
        self.vel.angular.z = 0
        self.pub.publish(self.vel)

    def main(self):
        while not rospy.is_shutdown():
            if self.current_goal < len(self.xd) and len(self.xd) == len(self.yd) and len(self.xd) == len(self.thetaRef):
                if sqrt((self.xd[self.current_goal]-self.x)**2 + (self.yd[self.current_goal]-self.y)**2) >= self.de or abs(self.thetaRef[self.current_goal]-self.theta) >= pi/180:

                    self.pub.publish(self.vel)

                    xe = self.x-self.xd[self.current_goal]
                    ye = self.y-self.yd[self.current_goal]

                    thetad = atan2(self.yd[self.current_goal]-self.y,self.xd[self.current_goal]-self.x)
                    thetae = self.theta - self.thetaRef[self.current_goal]
                    
                    if abs(thetae) > pi:
                        thetae = (thetae/abs(thetae))*(abs(thetae)-2*pi)

                    rho = sqrt(xe**2+ye**2)
                    alpha = thetad - self.theta

                    if abs(alpha) > pi:
                        alpha = (alpha/abs(alpha))*(abs(alpha)-2*pi)

                    beta = -thetae - alpha
                        
                    v = self.kp*rho
                    omega = self.ka*alpha + self.kb*beta

                    if v > 0.5:
                        v = 0.5
                    if omega > pi/8:
                        omega = pi/8
                    elif omega < -pi/8:
                        omega = -pi/8

                    #v = v*(1-tanh(abs(alpha)*2))

                    self.vel.linear.x = v
                    self.vel.angular.z = omega

                    #self.bag.write('Error', thetae)

                else:
                    rospy.loginfo('Next goal')
                    self.current_goal += 1

            else:
                rospy.signal_shutdown('Finished')

            rospy.on_shutdown(self.stop)

            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = trajectoryFollowing(0.7, 0.5, -0.2)
        node.main()

    except rospy.ROSInterruptException:
        pass    