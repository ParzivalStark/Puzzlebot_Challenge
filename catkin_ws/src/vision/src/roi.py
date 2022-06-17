#!/usr/bin/env python
from matplotlib.pyplot import contour
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from std_msgs.msg import Float64MultiArray
import numpy as np
#import matplotlib.pyplot as plt
from math import pi

from datetime import datetime

#from tensorflow.keras.models import load_model

#sudo systemctl restart puzzlebot.service

class LineFollowing:

    def __init__(self):
        rospy.init_node('ROI')
        rospy.loginfo("ROI initialized")
        self.kernelC = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        self.kernelO = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        self.bridge = CvBridge()
        self.rate = rospy.Rate(10)
        self.img_msg = Float64MultiArray()
        self.image = []
        self.pos = []
        self.angles = []
        self.xGoal = 0
        self.pastX = 0
        self.crrLines = [None, None, None]
        self.lastLines = [0, 0, 0]
        self.crrAngles = [0, 0, 0]
        self.lastAngles = [0, 0, 0]
        self.count = 0

        #self.model = load_model('src/vision/src/MySignsCNNv3.h5')
        #rospy.loginfo(self.model.summary())
        
        rospy.Subscriber('/video_source/raw', Image, self.image_callback)
        self.pub = rospy.Publisher('/imgs', Float64MultiArray, queue_size=20)
        #self.theta_e = rospy.Publisher("/theta_e", Float32, queue_size=10)
        #self.thetap_e = rospy.Publisher("/thetap_e", Float32, queue_size=10)
        #self.x_e = rospy.Publisher("/x_e", Float32, queue_size=10)
        #self.xp_e = rospy.Publisher("/xp_e", Float32, queue_size=10)
        #self.offset = rospy.Publisher("/offset", Float32, queue_size=10)

    def image_callback(self, msg):
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except:
            print('Error')

    def blobs(self, img, bgr):
        contours, _ = cv2.findContours(img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        rois = []
        newBlobs = np.zeros(img.shape)
        for i in range(len(contours)):
            area = cv2.contourArea(contours[i])
            if 800 > area > 400:
                cv2.drawContours(newBlobs, [contours[i]], 0, (255), cv2.FILLED)
        
        newBlobs = cv2.morphologyEx(newBlobs, cv2.MORPH_CLOSE, self.kernelC, iterations=6)
        
        contours, _ = cv2.findContours(img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        for i in range(len(contours)):
            area = cv2.contourArea(contours[i])
            if 1000 > area > 400:
                p = cv2.arcLength(contours[i], True)
                circ = 4*pi*(area/(p*p))

                if 0.2 < circ < 1.2:
                    x, y, w, h = cv2.boundingRect(contours[i])
                    #cv2.rectangle(bgr, (x, y), (x+w, y+h), (0, 255, 0), 3)
        return rois

    def blobs2(self, img, bgr):
        contours, _ = cv2.findContours(img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        rois = []

        for i in range(len(contours)):
            area = cv2.contourArea(contours[i])
            if 1700 > area > 300:
                p = cv2.arcLength(contours[i], True)
                circ = 4*pi*(area/(p*p))

                if 0.3 < circ < 1.2:
                    x, y, w, h = cv2.boundingRect(contours[i])
                    size = 0

                    if w > h:
                        size = w+50
                    else:
                        size = h+50

                    if x >= 25:
                        x -= 25
                    if y >= 25:
                        y -= 25

                    if x+size > img.shape[1]:
                        size = img.shape[1] - x
                    if y+size > img.shape[0]:
                        size = img.shape[0] - y
                    
                    cv2.rectangle(bgr, (x, y), (x+size, y+size), (0, 255, 0), 3)
                    cv2.imshow("ROIS", bgr)
                    rois.append([x,y,size])

        return rois

    def crop(self, img, rois):
        crops = []
        #rospy.loginfo("Image saved")
        for roi in rois:
            #now = datetime.now().strftime("%m_%d_%Y_%H_%M_%S")
            path = str(self.count)+'.png'
            self.count += 1
            #rospy.loginfo(path)
            x,y,size = roi  
            cropped_img = img[y:y+size,x:x+size]
            crops.append(cropped_img)
            #cv2.imwrite(path, cropped_img)
        return crops

    def edges(self, img):
        edit = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        edit = cv2.GaussianBlur(edit, (1, 1), 0)
        edit = cv2.Canny(edit, 50, 100, apertureSize=3)
        return edit

    def morphImg(self, img, i, j):
        morph = cv2.morphologyEx(img, cv2.MORPH_CLOSE, self.kernelC, iterations=i)
        morph = cv2.morphologyEx(morph, cv2.MORPH_OPEN, self.kernelO, iterations=j)
        return morph
    
    def prediction(self, im):
        if im is not None:
            edit = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY) 
            edit = cv2.blur(edit,(2,2)) 
            edit = cv2.resize(edit, (32, 32), interpolation = cv2.INTER_AREA) 

            #cv2.imshow("Signal", edit)

            list = edit.tolist()
            x, y = edit.shape
            data = []
            for i in range(x):
                for j in range(y):
                    data.append(list[i][j])
            
            self.img_msg.data = data
            #rospy.loginfo(type(self.img_msg.data[0][0]))
            self.pub.publish(self.img_msg)
        
        

    def main(self):
        rospy.loginfo("running")
        while not rospy.is_shutdown():
            if len(self.image) != 0:

                #cropped image to detect the road
                rimage = self.image.copy()
                rimage = rimage[0:(rimage.shape[0] - rimage.shape[0]/2), :]
                edit = cv2.resize(rimage,None,fx=0.5,fy=0.5,interpolation = cv2.INTER_AREA)
                #rgb = cv2.cvtColor(edit, cv2.COLOR_BGR2RGB)
                #hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
                
                #blue = cv2.inRange(hsv, (90, 70, 50), (128, 200, 200))
                #morphB = self.morphImg(blue, 1, 2)
                #morphB = cv2.resize(morphB,None,fx=0.5,fy=0.5,interpolation = cv2.INTER_AREA)

                #red1 = cv2.inRange(hsv, (0, 90, 70), (10, 255, 200))
                #red2 = cv2.inRange(hsv, (170, 90, 70), (180, 255, 200))
                #red  = cv2.bitwise_or(red1,red2)
                #morphR = self.morphImg(red, 4, 4)
                #morphR = cv2.resize(morphR,None,fx=0.5,fy=0.5,interpolation = cv2.INTER_AREA)

                edges = self.edges(rimage)
                edges = cv2.resize(edges,None,fx=0.5,fy=0.5,interpolation = cv2.INTER_AREA)

                #self.blobs(morphB, bgr)
                #self.blobs(morphR, bgr)
                rois = self.blobs2(edges, edit)

                rospy.loginfo(len(rois))
                crops = self.crop(edit, rois)

                #morphBGR = cv2.cvtColor(morph,cv2.COLOR_GRAY2BGR) 
                #edge = cv2.Canny(morph, 50, 100, apertureSize=3)

                #cv2.imshow('Blue', morphB)
                #scv2.imshow('Red', morphR)
                #cv2.imshow('Signs', bgr)

                for roi in crops:
                    self.prediction(roi)

            if cv2.waitKey(1) == 27:
                rospy.loginfo('Fin')
                break

            self.rate.sleep()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    cam = LineFollowing()
    cam.main()