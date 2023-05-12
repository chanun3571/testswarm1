#!/usr/bin/env python3
import sys
import cv2
import numpy as np
import time
import imutils
from matplotlib import pyplot as plt

# Functions
import HSV_filter as hsv
import shape_recognition as shape
import triangulation as tri
import calibration


import rospy
from sensor_msgs.msg import Image, CompressedImage
import cv2
from cv_bridge import CvBridge, CvBridgeError

rospy.init_node('opencv_example', anonymous=True)
bridge = CvBridge()

class color_shape_detection:
    #convert the ROS Image message to a CV2 Image
    def image_callback(self, img_msg):
        try:
            cv_image = bridge.compressed_imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError:
            rospy.loginfo("Failed")

    sub_image = rospy.Subscriber("/robot1/image/compressed", CompressedImage, image_callback)



    # Stereo vision setup parameters
    frame_rate = 30    #Camera frame rate (maximum at 120 fps)
    B = 6               #Distance between the cameras [cm]
    # f = 150            #Camera lense's focal length [mm]
    alpha = 45        #Camera field of view in the horisontal plane [degrees]




    # Main program loop with face detector and depth estimation using stereo vision


    while(True):
        frame_left = cv_image[:360,0:640]
        frame_right = img[:360,640:]

    ################## CALIBRATION #########################################################

        # frame_right, frame_left = calibration.undistortRectify(frame_right, frame_left)

    ########################################################################################

        # If cannot catch any frame, break
        if success==False:                    
            break

        else:
            # APPLYING HSV-FILTER:
            mask_right = hsv.add_HSV_filter(frame_right, 1)
            mask_left = hsv.add_HSV_filter(frame_left, 0)

            # Result-frames after applying HSV-filter mask
            res_right = cv2.bitwise_and(frame_right, frame_right, mask=mask_right)
            res_left = cv2.bitwise_and(frame_left, frame_left, mask=mask_left) 

            # APPLYING SHAPE RECOGNITION:
            circles_right = shape.find_circles(frame_right, mask_right)
            circles_left  = shape.find_circles(frame_left, mask_left)

            # Hough Transforms can be used aswell or some neural network to do object detection


            ################## CALCULATING BALL DEPTH #########################################################

            # If no ball can be caught in one camera show text "TRACKING LOST"
            if np.all(circles_right) == None or np.all(circles_left) == None:
                cv2.putText(frame_right, "TRACKING LOST", (75,50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255),2)
                cv2.putText(frame_left, "TRACKING LOST", (75,50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255),2)

            else:
                # Function to calculate depth of object. Outputs vector of all depths in case of several balls.
                # All formulas used to find depth is in video presentaion
                depth = tri.find_depth(circles_right, circles_left, frame_right, frame_left, B, alpha)

                cv2.putText(frame_right, "TRACKING", (75,50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (124,252,0),2)
                cv2.putText(frame_left, "TRACKING", (75,50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (124,252,0),2)
                cv2.putText(frame_right, "Distance: " + str(round(depth,3)), (200,50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (124,252,0),2)
                cv2.putText(frame_left, "Distance: " + str(round(depth,3)), (200,50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (124,252,0),2)
                # Multiply computer value with 205.8 to get real-life depth in [cm]. The factor was found manually.
                print("Depth: ", depth)                                            


            # Show the frames
            cv2.imshow("frame right", frame_right) 
            cv2.imshow("frame left", frame_left)
            cv2.imshow("mask right", mask_right) 
            cv2.imshow("mask left", mask_left)


            # Hit "q" to close the window
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break


    # Release and destroy all windows before termination
    cap.release()

    cv2.destroyAllWindows()

while not rospy.is_shutdown():
    rospy.spin()