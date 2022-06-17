#!/usr/bin/env python
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
import numpy as np

class Camera:

    def __init__(self):
        rospy.init_node('Camera')

        self.bridge = CvBridge()
        self.rate = rospy.Rate(10)
        self.image = []
        self.go = Bool()

        rospy.Subscriber('/video_source/raw', Image, self.image_callback)
        self.pub = rospy.Publisher("/go", Bool, queue_size=10)

    def contours(self):
        gray  = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        for i in range(3):
            blur  = cv2.GaussianBlur(self.image[:,:,i], (7, 7), 0)
            canny = cv2.Canny(blur, 50, 100, 0)

            if (i == 0):
                filter = canny.copy()
            else:
                filter = cv2.add(filter, canny)
    
        negFilter = cv2.bitwise_not(filter)

        edges = cv2.cvtColor(gray, cv2.COLOR_GRAY2RGB)
        edges[:,:,2] = cv2.add(edges[:,:,0], filter)
        edges[:,:,0] = cv2.bitwise_and(edges[:,:,1], negFilter)
        edges[:,:,1] = cv2.bitwise_and(edges[:,:,2], negFilter)

        return edges
    
    def mask(self, img, mask):
       
        copy = img
        masked = cv2.bitwise_and(copy, copy, mask = mask)

        return masked

    def detectColor(self, img, lower, upper):

        kernelC = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7,7))
        kernelO = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))

        copy = img.copy()
        hsv  = cv2.cvtColor(copy, cv2.COLOR_BGR2HSV)

        lower_ = np.array(lower)
        upper_ = np.array(upper)

        mask = cv2.inRange(hsv, lower_, upper_)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernelO, iterations = 2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernelC, iterations = 1)
        
        
        return mask

    def redLight(self):
        red_copy = self.image.copy()
        hsv  = cv2.cvtColor(red_copy, cv2.COLOR_BGR2HSV)

        lower_red = np.array([0,100,100])
        upper_red = np.array([10,255,255])

        #blur = cv.GaussianBlur(hsv, (5,5), 3)

        mask = cv2.inRange(hsv, lower_red, upper_red)

        lower_red = np.array([170,100,100])
        upper_red = np.array([180,255,255])

        mask = mask + cv2.inRange(hsv, lower_red, upper_red) 
        #mask = cv.morphologyEx(mask,cv.MORPH_CLOSE,(3,3), iterations = 2)
        red  = cv2.bitwise_and(red_copy,red_copy, mask= mask)
        return red
    
    def greenLight(self):
        green_copy = self.image.copy()
        hsv  = cv2.cvtColor(green_copy, cv2.COLOR_BGR2HSV)

        lower_green = np.array([40,80,100])
        upper_green = np.array([70,255,255])

        #blur = cv.GaussianBlur(hsv, (5,5), 3)

        mask = cv2.inRange(hsv, lower_green, upper_green)
        #mask = cv.morphologyEx(mask,cv.MORPH_CLOSE,(3,3), iterations = 2)
        green  = cv2.bitwise_and(green_copy,green_copy, mask= mask)
        return green
    
    def circleDetection(self, input):
        go = False
        circlesRound = None

        height, width = input.shape[:2]
        maxRadius = int(5*(width/12))
        minRadius = int(0.10*(width/12))
        #bgr = cv2.cvtColor(input, cv2.COLOR_HSV2BGR)
        #gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

        circles = cv2.HoughCircles(image=input,
                                    method=cv2.HOUGH_GRADIENT,
                                    dp=2,
                                    minDist=2*minRadius,
                                    param1=100,
                                    param2=80,
                                    minRadius=minRadius,
                                    maxRadius=maxRadius)

        if circles is not None:
            go = True
            circlesRound = np.round(circles[0, :]).astype("int")
            
        return circlesRound, go

    
    def image_callback(self, msg):
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except:
            print('Error')

    def drawCircles(self, img, circles):
        output = img.copy()
        if circles is not None:
            for (x,y,r) in circles:
                cv2.circle(output,(x,y),r,(255,255,255),4)

        return output

    def main(self):
        while not rospy.is_shutdown():

            if len(self.image) != 0:
                copy = self.image.copy()
                red1 = self.detectColor(copy, [0,100,100], [10,255,255])
                red2 = self.detectColor(copy, [170,100,100], [180,255,255])
                red  = cv2.bitwise_or(red1, red2) 
                green = self.detectColor(copy, [40,80,80], [70,255,255])
                #cv2.imshow('red', red)
                #cv2.imshow('green', green)

                rCircles, rBool = self.circleDetection(red)
                gCircles, gBool = self.circleDetection(green)

                if rBool:
                    self.go.data = False
                    self.pub.publish(self.go)
                elif gBool:
                    self.go.data = True
                    self.pub.publish(self.go)

                cv2.imshow('Red circles', self.drawCircles(self.mask(copy, red), rCircles))
                cv2.imshow('Green circles', self.drawCircles(self.mask(copy, green), gCircles))
    
            if cv2.waitKey(1) == 27:
                rospy.loginfo('Fin')
                break

            self.rate.sleep()
        cv2.destroyAllWindows()

    #cv2.destroyAllWindows()

if __name__ == '__main__':
    cam = Camera()
    cam.main()
