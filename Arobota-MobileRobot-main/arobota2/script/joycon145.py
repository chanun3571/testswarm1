#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import rospy
import numpy as np
from geometry_msgs.msg import Pose,Twist,PoseStamped
from sensor_msgs.msg import Joy
import tf
global joy_msg
global robot_ang
global robot_pos
global refpoint
global centerref
global theta
global radius
global max_speed 
global velocity
global initcomp
global pub1
global pub4
global pub5
global colmax
global colmin

robot_ang=np.array([0.0,0.0,0.0])
robot_pos=np.array([[0.0,0.0],[0.0,0.0],[0.0,0.0]])
refpoint=np.array([[0.0,0.0,0.0],[0.0,0.0],[0.0,0.0]])
centerref=np.array([0.0,0.0])
initcomp=0
max_speed = 1
colmax=0.8
colmin=0.3
radius=0.5
theta=0.0
d_ang = 0
joy_msg = [0]*8
velocity=[Twist(),Twist(),Twist()]
pub1 = rospy.Publisher('/scamper201/scamper_driver/cmd_robot_vel', Twist, queue_size=10)
pub4 = rospy.Publisher('/scamper204/scamper_driver/cmd_robot_vel', Twist, queue_size=10)
pub5 = rospy.Publisher('/scamper205/scamper_driver/cmd_robot_vel', Twist, queue_size=10)


def refpointcalc():
    global centerref
    global radius
    global joy_msg
    global theta
    global refpoint
    centerref=np.array([-joy_msg[0],joy_msg[1]])/30.0+centerref
    radius+=joy_msg[4]/30.0
    if radius>2:
        radius=2
    if radius<0.3:
        radius=0.3
    theta+=joy_msg[3]/30.0/radius
    if theta>2*math.pi:
        theta-=2*math.pi
    if theta<-2*math.pi:
        theta-=2*math.pi
    refpoint[0]=centerref+radius*np.array([math.cos(theta),math.sin(theta)])
    refpoint[1]=centerref+radius*np.array([math.cos(theta+math.pi*2/3),math.sin(theta+math.pi*2/3)])
    refpoint[2]=centerref+radius*np.array([math.cos(theta+math.pi*4/3),math.sin(theta+math.pi*4/3)])
    log=np.array([centerref[0],centerref[1],radius,theta])
    log=np.round(log,2)
    print(log)

def velcalc():
    global refpoint
    global velocity
    global robot_ang
    global robot_pos

    ColV=[CollisionAvoid(0,1),CollisionAvoid(1,2),CollisionAvoid(2,0)]
    glx=[refpoint[0][0]-robot_pos[0][0]-ColV[0][0]+ColV[2][0],refpoint[1][0]-robot_pos[1][0]-ColV[1][0]+ColV[0][0],refpoint[2][0]-robot_pos[2][0]-ColV[2][0]+ColV[1][0]]
    gly=[refpoint[0][1]-robot_pos[0][1]-ColV[0][1]+ColV[2][1],refpoint[1][1]-robot_pos[1][1]-ColV[1][1]+ColV[0][1],refpoint[2][1]-robot_pos[2][1]-ColV[2][1]+ColV[1][1]]
    
    
    lcz=robot_ang

    for i in range(0,3):
        velabs=glx[i]*glx[i]+gly[i]*gly[i]
        if velabs>max_speed*max_speed:
            glx[i]=glx[i]/velabs*max_speed
            gly[i]=gly[i]/velabs*max_speed
        velocity[i].linear.x=glx[i]*math.cos(lcz[i])+gly[i]*math.sin(lcz[i])
        velocity[i].linear.y=-glx[i]*math.sin(lcz[i])+gly[i]*math.cos(lcz[i])
        velocity[i].angular.z=-2*lcz[i]

def Ref():
    global velocity
    # set subscriber and publisher
    global pub1
    global pub4
    global pub5
    joy = rospy.Subscriber('joy',Joy,JoyCallback) #joy
    rospy.Subscriber('/vrpn_client_node/scamper203/pose', PoseStamped, poseCallback1) #add
    rospy.Subscriber('/vrpn_client_node/scamper204/pose', PoseStamped, poseCallback4) #add
    rospy.Subscriber('/vrpn_client_node/scamper205/pose', PoseStamped, poseCallback5) #add

    
    # prepare pose message to publish
    rospy.init_node('ref',anonymous=True)
    r = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        refpointcalc()
        velcalc()  
        pub1.publish(velocity[0])
        pub4.publish(velocity[1])
        pub5.publish(velocity[2])
        r.sleep()

def JoyCallback(msg):
    global joy_msg
    if initcomp==0:
        joy_msg = msg.axes
    return 0

def poseCallback1(data):
    global robot_ang
    global robot_pos
    Z=data.pose.orientation.z
    W=data.pose.orientation.w
    robot_ang[0]=math.atan2(2*Z*W,2*W*W-1)
    robot_pos[0][0]=data.pose.position.x
    robot_pos[0][1]=data.pose.position.y
def poseCallback4(data):
    global robot_ang
    global robot_pos
    Z=data.pose.orientation.z
    W=data.pose.orientation.w
    robot_ang[1]=math.atan2(2*Z*W,2*W*W-1)
    robot_pos[1][0]=data.pose.position.x
    robot_pos[1][1]=data.pose.position.y
def poseCallback5(data):
    global robot_ang
    global robot_pos
    Z=data.pose.orientation.z
    W=data.pose.orientation.w
    robot_ang[2]=math.atan2(2*Z*W,2*W*W-1)
    robot_pos[2][0]=data.pose.position.x
    robot_pos[2][1]=data.pose.position.y

def CollisionAvoid(a,b):
    global robot_pos
    global colmax
    global colmin
    pos=robot_pos[a]-robot_pos[b]
    d=math.sqrt(pos[0]*pos[0]+pos[1]*pos[1])
    if d<colmax:
        if d>colmin:
            k=5/(colmax-colmin)/(colmax-colmin)*(d-colmax)*(d-colmax)
        else:
            k=5
        return (robot_pos[b]-robot_pos[a])*k
    else:
        return [0.0,0.0]

if __name__ == '__main__':
    try:
        Ref()
    except rospy.ROSInterruptException: pass
