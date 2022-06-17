#!/usr/bin/env python3

import rospy
import cv2
from std_msgs.msg import Float64MultiArray
import numpy as np

import os
os.environ["CUDA_VISIBLE_DEVICES"]="-1" 
import tensorflow as tf
from tensorflow.keras.models import load_model


class SignalRecognition:

    def __init__(self):
        rospy.init_node('MyNet')
        rospy.loginfo("MyNet initialized")
        rospy.sleep(3)
        
        self.rate = rospy.Rate(100)

        self.img = []

        self.classes = ['Yellow', 'Stop', 'Green', 'Red', 'SpeedLimit', 'Right', 'Forward']
        
        self.model = load_model('src/vision/src/MySignsCNNv4.h5')
        rospy.loginfo(self.model.summary())

        rospy.Subscriber('/imgs', Float64MultiArray, self.image_callback)
    
    def image_callback(self, img_msg):
        
        self.img = []
        for i in range(32):
            row = []
            for j in range(32):
                row.append(img_msg.data[32*i+j])
            self.img.append(row)
        #cv2.imshow("Signal", np.array(self.img))
        rospy.loginfo(self.prediction())


    def prediction(self):

        x = np.array(self.img)
        x = x.reshape(1, 32, 32, 1) 
        x = x/255.

        results = self.model.predict(x)
        y_pred = np.argmax(results, axis=-1)
        #rospy.loginfo(results[0][int(y_pred)])
        if results[0][int(y_pred)] > 0.5:
            return self.classes[int(y_pred)]
        return None
    
    def main(self):
        while not rospy.is_shutdown:
            rospy.loginfo("running")
            rospy.loginfo(self.prediction())
            self.rate.sleep()
        rospy.spin()
    
if __name__ == '__main__':
    #gpus = tf.config.list_physical_devices('GPU')
    #tf.config.set_logical_device_configuration(gpus[0],[tf.config.LogicalDeviceConfiguration(memory_limit=64)])
    Signal = SignalRecognition()
    Signal.main()

    

        
        