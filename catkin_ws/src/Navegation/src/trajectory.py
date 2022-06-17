#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from math import sqrt
import numpy as np

class Trajectory:

    def __init__(self):
        rospy.init_node("Trajectory")
        rospy.loginfo('Iniciando')
        self.pub = rospy.Publisher("/step", Twist, queue_size=10)

        rospy.Subscriber("/position", Twist, self.pos_callback)
        rospy.Subscriber("/goal", Twist, self.goal_callback)

        self.rate = rospy.Rate(10)
 
        #initial conditions in X
        self.x0 = 0
        self.xf = 0
        self.vx0 = 0
        self.vxf = 0
        self.ax0 = 0
        self.axf = 0

        #initial conditions Y
        self.y0 = 0
        self.yf = 0
        self.vy0 = 0
        self.vyf = 0
        self.ay0 = 0
        self.ayf = 0

        #Finaltime
        self.Tf = 10
        #flag
        self.flag = False
        #initialize step
        self.step = Twist()
        self.step.linear.x = 0
        self.step.linear.y = 0
        self.step.linear.z = 0
        self.step.angular.x = 0
        self.step.angular.y = 0
        self.step.angular.z = 0

    def pos_callback(self,pos):
        self.x0 = pos.linear.x
        self.y0 = pos.linear.y
    
    def goal_callback(self,goal):
        if self.xf != goal.linear.x or self.yf != goal.linear.y:
            self.xf = goal.linear.x
            self.yf = goal.linear.y
            #update final time acording to the distance
            #velocity is 30 cm / sec
            dist = sqrt((self.xf-self.x0)**2 + (self.yf-self.y0)**2)
            self.Tf = dist*2

            self.Px = self.GenPoly(self.x0,self.xf,self.vx0,self.vxf,self.ax0,self.axf,self.Tf)
            self.Py = self.GenPoly(self.y0,self.yf,self.vy0,self.vyf,self.ay0,self.ayf,self.Tf)
            self.flag = True
            self.t0 = rospy.get_time()

    def GenPoly(self,x0,xf,v0,vf,a0,af,Tf):
        
        M = np.array([[0,0,0,0,0,1],
            [Tf**5,Tf**4,Tf**3,Tf**2,Tf,1],
            [0,0,0,0,1,0],
            [5*Tf**4,4*Tf**3,3*Tf**2,2*Tf,1,0],
            [0,0,0,2,0,0],
            [20*Tf**3,12*Tf**2,6*Tf,1,0,0]])

        X = np.array([x0,xf,v0,vf,a0,af])

        P = np.matmul(np.linalg.inv(M),X)
        return P
    
    def main(self):
        rospy.loginfo('running')
        t = 0
        while not rospy.is_shutdown():
            if self.flag and t < self.Tf:
                #time
                currentTime = rospy.get_time()
                t = currentTime - self.t0
                
                self.step.linear.x = self.Px[0]*t**5 + self.Px[1]*t**4 + self.Px[2]*t**3 + self.Px[3]*t**2 + self.Px[4]*t + self.Px[5]
                self.step.linear.y = self.Py[0]*t**5 + self.Py[1]*t**4 + self.Py[2]*t**3 + self.Py[3]*t**2 + self.Py[4]*t + self.Py[5]

                self.pub.publish(self.step)
            else:
                self.flag = False
                t = 0
            self.rate.sleep()


        
if __name__ == '__main__':
    node = Trajectory()
    node.main()

