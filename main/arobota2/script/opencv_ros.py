#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image, CompressedImage

# Import OpenCV libraries and tools
import cv2
from cv_bridge import CvBridge, CvBridgeError

# Initialize the ROS Node named 'opencv_example', allow multiple nodes to be run with this name
rospy.init_node('opencv_example', anonymous=True)

# Print "Hello ROS!" to the Terminal and to a ROS Log file located in ~/.ros/log/loghash/*.log
# rospy.loginfo("Hello ROS!")

# Initialize the CvBridge class
bridge = CvBridge()

# Define a function to show the image in an OpenCV Window
def show_image(img):
    left = img[:360,0:640]
    right = img[:360,640:]
    cv2.imshow('left cam',left)
    cv2.imshow('right cam',right)
    cv2.waitKey(3)

# Define a callback for the Image message
def image_callback(img_msg):
    # log some info about the image topic
    # rospy.loginfo(img_msg.header)

    # Try to convert the ROS Image message to a CV2 Image
    try:
        cv_image = bridge.compressed_imgmsg_to_cv2(img_msg, "bgr8")
    except CvBridgeError:
        rospy.loginfo("Failed")

    # Flip the image 90deg
    # cv_image = cv2.transpose(cv_image)
    # cv_image = cv2.flip(cv_image,1)

    # Show the converted image
    show_image(cv_image)

# Initalize a subscriber to the "/camera/rgb/image_raw" topic with the function "image_callback" as a callback
sub_image = rospy.Subscriber("/robot1/image/compressed", CompressedImage, image_callback)

# Loop to keep the program from shutting down unless ROS is shut down, or CTRL+C is pressed
while not rospy.is_shutdown():
    rospy.spin()

