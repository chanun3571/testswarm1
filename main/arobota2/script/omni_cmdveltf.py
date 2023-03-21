#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Twist 
from math import sin, cos, pi, sqrt

class TwistToVel():
    def __init__(self):
        rospy.init_node("twist_to_velocity")
        # self.pub_lmotor = rospy.Publisher('lwheel_vtarget', Float32,queue_size=10)
        # self.pub_rmotor = rospy.Publisher('rwheel_vtarget', Float32,queue_size=10)
        self.pub_motor = rospy.Publisher('wheel_vtarget', String,queue_size=10)
        rospy.Subscriber('cmd_vel', Twist, self.twistCallback)
        self.v_center = 0
        self.v_left = 0
        self.v_right = 0
   
    def twistCallback(self,msg):
        self.r = 125/1000 #m
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.vr = msg.angular.z
        self.v_center = ((1/3)*self.r*self.vr - (2/3)*self.vy)
        self.v_right = ((1/3)*self.r*self.vr +(sqrt(3)/3)*self.vx + (1/3)*self.vy)
        self.v_left = ((1/3)*self.r*self.vr -(sqrt(3)/3)*self.vx + (1/3)*self.vy)
        x = str(self.v_left)+","+str(self.v_center)+","+str(self.v_right)
        self.pub_motor.publish(x)

if __name__ == '__main__':
    """ main """
    TwistToVel()
    rospy.spin()