#!/usr/bin/env python3
import cv2
import numpy as np
import rospy


class webcam():
    def __init__(self):
        rospy.init_node('camera')  
        self.all_camera_idx_available = []
        for self.camera_idx in range(10):
            self.cap = cv2.VideoCapture(self.camera_idx)

    def camerafeed(self):

            if self.cap.isOpened():
                print(f'Camera index available: {self.camera_idx}')
                self.all_camera_idx_available.append(self.camera_idx)
                self.cap.release()

if __name__ == '__main__':
    try:
        webcam()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished")