import sys
import cv2
import numpy as np
import time


def add_HSV_filter(frame, camera):

	# Blurring the frame
    blur = cv2.GaussianBlur(frame,(25,25),0) 
    # blur = cv2.bilateralFilter(frame,9,75,75)
    # Converting RGB to HSV
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

    # l_b_r = np.array([10, 100, 150])        # Lower limit for red ball
    # u_b_r = np.array([30, 255, 255])       # Upper limit for red ball
    # l_b_l = np.array([10, 100, 150])        # Lower limit for red ball
    # u_b_l = np.array([30, 255, 255])       # Upper limit for red ball

    # l_b_r = np.array([145, 100, 150])        # Lower limit for red ball
    # u_b_r = np.array([155, 255, 255])       # Upper limit for red ball
    # l_b_l = np.array([145, 100, 150])        # Lower limit for red ball
    # u_b_l = np.array([155, 255, 255])       # Upper limit for red ball

    l_b_r = np.array([100, 100, 100])        # Lower limit for pink ball
    u_b_r = np.array([180, 255, 225])       # Upper limit for pink ball
    l_b_l = np.array([100, 100, 100])        # Lower limit for pink ball
    u_b_l = np.array([180, 255, 225])       # Upper limit for pink ball

    # l_b_r = np.array([140, 106, 20])        # Lower limit for blue ball
    # u_b_r = np.array([160, 255, 255])       # Upper limit for blue ball
    # l_b_l = np.array([140, 106, 20])        # Lower limit for blue ball
    # u_b_l = np.array([160, 255, 255])       # Upper limit for blue ball

	# HSV-filter mask
    # mask = cv2.inRange(hsv, l_b_l, u_b_l)

    if(camera == 1):
        mask = cv2.inRange(hsv, l_b_r, u_b_r)
        # print(hsv)
    else:
        mask = cv2.inRange(hsv, l_b_l, u_b_l)


    # Morphological Operation - Opening - Erode followed by Dilate - Remove noise
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    return mask