#!/usr/bin/env python
from cv2 import imshow
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from std_msgs.msg import Float32
import numpy as np
#import matplotlib.pyplot as plt
from math import cos,sin,atan2

#sudo systemctl restart puzzlebot.service

class LineFollowing:

    def __init__(self):
        rospy.init_node('Line_Following')
        
        rospy.loginfo("Line Following initialized")

        self.bridge = CvBridge()
        self.rate = rospy.Rate(80)
        self.image = []
        self.pos = []
        self.angles = []
        self.xGoal = 0
        self.pastX = 0
        self.turn = 0
        self.crrLines = [None, None, None]
        self.lastLines = [0, 0, 0]
        self.crrAngles = [0, 0, 0]
        self.lastAngles = [0, 0, 0]
        self.lines = True

        rospy.Subscriber('/video_source/raw', Image, self.image_callback)
        self.theta_e = rospy.Publisher("/theta_e", Float32, queue_size=10)
        self.thetap_e = rospy.Publisher("/thetap_e", Float32, queue_size=10)
        self.turn_pub = rospy.Publisher('/turn', Float32, queue_size=10)
        self.x_e = rospy.Publisher("/x_e", Float32, queue_size=10)
        self.xp_e = rospy.Publisher("/xp_e", Float32, queue_size=10)
        self.offset = rospy.Publisher("/offset", Float32, queue_size=10)
        rospy.Subscriber('/turn', Float32, self.turn_callback)


    def image_callback(self, msg):
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except:
            print('Error')

    def turn_callback(self, turn):
        self.turn = turn.data

    def blobs(self, image):
        img = cv2.bitwise_not(image)
        img, _ = cv2.findContours(img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        return img

    def top_blobs(self, contours):
        top_blobs = []
        j = 0
        for i in range(len(contours)):
            p = cv2.arcLength(contours[i], True)
            if p > 200:
                
                if j == 0:
                    top_blobs.append([i,p])
                    j += 1
                elif j < 3:
                    j_ = j
                    for k in range(j):
                        if p >= top_blobs[k][1]:
                            top_blobs.insert(k, [i,p])
                            j += 1
                            break
                    if j_ == j:
                        top_blobs.insert(k, [i,p])
                        j += 1
                else:
                    for k in range(j):
                        if p >= top_blobs[k][1]:
                            top_blobs.insert(k, [i,p])
                            top_blobs.pop()
        res = []
        for i, p in top_blobs:
            res.append(contours[i])
            
        return res
        
    def getLines(self, contours, shape):
        rows,cols = shape #rows,cols = img.shape[:2]
        lines = []
        a = []
        b = []
        #cv2.drawContours(res, contours, -1, (0,0,255), 3)
        for contour in contours:
            [vx,vy,x,y] = cv2.fitLine(contour, cv2.DIST_L2,0,0.01,0.01)
            lines.append([vx,vy,x,y])
            #lefty = int((-x*vy/vx) + y)
            #righty = int(((cols-x)*vy/vx)+y)
            #cv2.line(res,(cols-1,righty),(0,lefty),(0,255,0),2)
            #cv2.imshow('Lines', res)
        return lines

    

    def intersect(self, lines, center, height):
        a = []
        b = []
        x = 0
        y = 0
        count = 0

        if self.turn == 0:
            if len(lines) > 1:
                for line in lines:
                    a.append(-(line[1]/line[0]))
                    b.append(line[3] + (line[1]/line[0])*line[2])

                for i in range(len(a)):
                    for j in range(i, len(a)):
                        if a[i] != a[j]:
                            count += 1
                            x += (b[j]-b[i])/(a[i]-a[j])
                            y += a[i]*(b[j]-b[i])/(a[i]-a[j]) + b[i]
                
                if count != 0:
                    y = y/count
                    self.pastX = self.xGoal
                    self.xGoal = x/count - center
                    if y < 0:
                        self.xGoal = -self.xGoal
        elif self.turn == 1:
            self.xGoal = 0
            self.pastX = self.xGoal
        elif self.turn == 2:
            self.xGoal = center*3
            self.pastX = self.xGoal
        
        if len(lines) < 2:
            self.lines = False
        else:
            if not self.lines:
                #self.turn_pub.publish((Float32)(0))
                self.lines = True
        
        #cv2.line(res,(int(self.xGoal+center),0),(int(self.xGoal+center),200),(255,0,0),2)
        #cv2.imshow('Punto de Fuga', res)

        

    def lineChar(self, lines):
        angles_ = []
        self.angles = []
        pos_ = []
        self.pos = []
        for line in lines:
            angles_.append(atan2(line[1], line[0]))
            x = (line[2] - (line[3]/line[1])*line[0])
            pos_.append(x)
            #cv2.line(res,(x,0),(x,line[3]),(0,0,255),5)

        for i in range(len(pos_)):
            if i == 0:
                self.angles.append(angles_[i])
                self.pos.append(pos_[i])
            else:
                found = False
                for k in range(i):
                    if pos_[i] >= self.pos[k]:
                        self.angles.insert(k, angles_[i])
                        self.pos.insert(k, pos_[i])
                        found = True
                        break
                if not found:
                    self.angles.append(angles_[i])
                    self.pos.append(pos_[i])

        #cv2.imshow('PosX', res)

    def lineTracking(self, kOff):
        offset = 0
        cant = len(self.pos)
        mdi = np.zeros(cant)

        if cant == 3:
            mdi = [0, 1, 2]
        else:
            md = np.zeros(cant)
            for j in range(cant):
                matched = False
                for i in range(3):
                    if self.lastLines[i] <= self.pos[j]:
                        if i == 0:
                            md[j] = self.pos[j] - self.lastLines[i]
                            mdi[j] = 0
                        else:
                            left = self.pos[j] - self.lastLines[i]
                            right = self.pos[j] - self.lastLines[i-1]
                            if left < abs(right):
                                md[j] = left
                                mdi[j] = i
                            else:
                                md[j] = right
                                mdi[j] = i-1
                        matched = True
                        break                
                if not matched:
                    md[j] = self.pos[j] - self.lastLines[i]
                    mdi[j] = i

            if cant == 2 and mdi[0] == mdi[1]:
                if mdi[0] == 1:
                    if abs(md[0]) < abs(md[1]):
                        mdi[1] -= md[1]/abs(md[1])
                    elif abs(md[0]) == abs(md[1]) and md[0]*md[1] < 0:
                        if abs(self.lastLines[0] - self.pos[0]) > abs(self.lastLines[2] - self.pos[1]):
                            mdi[1] += 1
                        else:
                            mdi[0] -= 1
                    else:
                        mdi[0] -= md[0]/abs(md[0])
                else:
                    marker = - (mdi[0] - 1)
                    if marker > 0:
                        mdi[1] += 1
                    else:
                        mdi[0] -= 1

        self.crrLines = [None, None, None]

        for i in range(cant):
            self.lastLines[int(mdi[i])] = self.pos[i]
            self.crrLines[int(mdi[i])] = self.pos[i]
            self.lastAngles[int(mdi[i])] = self.crrAngles[int(mdi[i])]
            self.crrAngles[int(mdi[i])] = self.angles[i]

            #cv2.line(res,(self.crrLines[int(mdi[i])],0),(self.crrLines[int(mdi[i])],200),(mdi[i]*125,0,255-mdi[i]*125),5)
            
            offset -= mdi[i]-1
        
        if self.crrLines[1] != None:
            offset *= 2.5 #2

        if self.turn != 0:
            offset = 0

        self.xGoal -= offset*kOff

        

        self.offset.publish((Float32)(offset))

        #cv2.imshow('Lines', res)



    def getErrors(self, center):
        rospy.loginfo(self.crrAngles)
        if self.crrLines[0] != None and self.crrLines[2] != None:
            if self.crrLines[1] != None:
                thetae  = (Float32)(self.crrAngles[1])
                thetape = (Float32)(self.crrAngles[1] - self.lastAngles[1])
                xe      = (Float32)(self.crrLines[1] - center)
            else:
                thetae  = (Float32)((self.crrAngles[0]+self.crrAngles[2])/2)
                thetape = (Float32)(((self.crrAngles[0]+self.crrAngles[2]) - (self.lastAngles[0]+self.lastAngles[2]))/2)
                xe      = (Float32)((self.crrLines[0] + self.crrLines[2])/2 - center)
                
            self.theta_e.publish(thetae)
            self.thetap_e.publish(thetape)
            self.x_e.publish(xe)

    def morphBlobs(self, img):
        edit = cv2.medianBlur(img, 5)
        edit = cv2.cvtColor(edit, cv2.COLOR_BGR2GRAY)
        edit = cv2.threshold(edit, 100, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)[1]
        edit = cv2.resize(edit,None,fx=0.5,fy=0.5,interpolation = cv2.INTER_AREA)
        edit = edit[(edit.shape[0] - edit.shape[0]/2):, :]
        return edit

    def main(self):
        kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, (7, 7))
        while not rospy.is_shutdown():

            if len(self.image) != 0:

                #cropped image to detect the road
                edit = self.image.copy()
                edit = cv2.medianBlur(edit, 5)
                edit = cv2.cvtColor(edit, cv2.COLOR_BGR2GRAY)
                edit = cv2.threshold(edit, 100, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)[1]
                edit = cv2.resize(edit,None,fx=0.5,fy=0.5,interpolation = cv2.INTER_AREA)
                edit = edit[(edit.shape[0] - edit.shape[0]/2):, :]
                
                edit = cv2.morphologyEx(edit, cv2.MORPH_CLOSE, kernel, iterations=2)
                edit = cv2.morphologyEx(edit, cv2.MORPH_OPEN, kernel, iterations=2)
                
                #morphBGR = cv2.cvtColor(edit,cv2.COLOR_GRAY2BGR) 
                #edge = cv2.Canny(morph, 50, 100, apertureSize=3)
                blob = self.blobs(edit)
                top_blobs = self.top_blobs(blob)
                lines = self.getLines(top_blobs, edit.shape[:2])
                self.intersect(lines, edit.shape[1]/2, edit.shape[0])
                self.lineChar(lines)
                self.lineTracking(edit.shape[1]/2) #/6
                rospy.loginfo(self.xGoal)
                rospy.loginfo(self.turn)
                self.x_e.publish(self.xGoal)
                self.xp_e.publish(self.xGoal - self.pastX)
                #self.getErrors(morph.shape[1]/2)

                #cv2.imshow('Blobs',morph)
                #cv2.imshow('Lines', morphBGR)

            if cv2.waitKey(1) == 27:
                rospy.loginfo('Fin')
                break

            self.rate.sleep()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    cam = LineFollowing()
    cam.main()
