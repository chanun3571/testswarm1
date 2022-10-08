#!/usr/bin/env python

import getch
import rospy
from std_msgs.msg import String#String message 
from std_msgs.msg import Int8
from geometry_msgs.msg import Vector3

pub = rospy.Publisher('key', Vector3, queue_size=10) # "key" is the publisher name


def keys():
    global omega
    global x
    global y
    rospy.init_node('keypress',anonymous=True)
    rate = rospy.Rate(10) #try removing this line ans see what happens
    k=ord(getch.getch()) 
    if k==119:
        button = "up"
        y = 1
    if k==97:
        button = "left"
        x = -1
    if k==100:
        button = "right"
        x = 1
    if k==115:
        button = "back"
        y= -1
    if k==113:
        button = "ccw"
        omega = -1
    if k==101:
        button = "cw"
        omega = 1
    else: 
        button = "" 
        x = 0
        y = 0
        omega = 0
    rospy.loginfo(button)
    #w=119, a=97, d=100, s=115, q=113, e=101

def motion(w,vx,vy):
    r = (45/2)/100 #m
    d = 60/100 #m
    print(w,vx,vy)
    u1 = 1/r*(-d*w + vx)
    u2 = 1/r*(-d*w -1/2*vx -0.866*vy)
    u3 = 1/r*(-d*w -1/2*vx +0.866*vy)
    ros_translation = Vector3()
    ros_translation.x = u1
    ros_translation.y = u2
    ros_translation.z = u3
    pub.publish(ros_translation)
    rospy.loginfo(ros_translation)

def spin():
    # initialize message
    while not rospy.is_shutdown():
        keys()
        motion(omega,x,y)


if __name__=='__main__':
    try:
        spin()
    except rospy.ROSInterruptException:
        pass