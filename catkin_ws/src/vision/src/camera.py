#!/usr/bin/env python
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class Camera:

    def __init__(self):
        rospy.init_node('Camera')

        self.bridge = CvBridge()
        self.rate = rospy.Rate(10)
        self.image = []

        sub = rospy.Subscriber('/video_source/raw', Image, self.image_callback)

        #img_back = bridge.cv2_to_imgmsg(result_image)

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
    
    def image_callback(self, msg):
        try:
            rospy.loginfo('New frame')
            self.image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except:
            print('Error')

    def main(self):
        i=0
        while not rospy.is_shutdown():
            rospy.loginfo('Ciclo')
            if len(self.image) != 0:   
                i += 1 
                path = './image'+str(i)+'.png'
                rospy.loginfo('imagen')
                cv2.imshow('Camera', self.contours())
                #cv2.imwrite(path,self.image)
            if cv2.waitKey(1) == 27 or i>10:
                rospy.loginfo('Fin')
                break

            self.rate.sleep()
        cv2.destroyAllWindows()

    #cv2.destroyAllWindows()

if __name__ == '__main__':
    cam = Camera()
    cam.main()
