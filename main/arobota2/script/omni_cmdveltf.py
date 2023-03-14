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
        #self.wheelrad = 24 * (10**-3) #wheelradius=24 mm
        #self.base = rospy.get_param("~base_width", 200*(10^-3)) #radius=100 mm
   
    def twistCallback(self,msg):
        self.r = 100/1000 #m
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.vr = msg.angular.z
        self.v_center = ((1/3)*self.r*self.vr - (2/3)*self.vy)
        self.v_right = ((1/3)*self.r*self.vr +(sqrt(3)/3)*self.vx + (1/3)*self.vy)
        self.v_left = ((1/3)*self.r*self.vr -(sqrt(3)/3)*self.vx + (1/3)*self.vy)
        # self.v_center = (self.r*self.vr - self.vy)
        # self.v_right = (self.r*self.vr +cos(pi/6)*self.vx +sin(pi/6)*self.vy)
        # self.v_left = (self.r*self.vr -cos(pi/6)*self.vx + sin(pi/6)*self.vy)
        x = str(self.v_left)+","+str(self.v_center)+","+str(self.v_right)
        self.pub_motor.publish(x)

        # u1 = (-self.base*self.dr + self.dx)
        # u2 = (-self.base*self.dr -cos(pi/3)*self.dx -sin(pi/3)*self.dy)
        # u3 = (-self.base*self.dr -cos(pi/3)*self.dx + sin(pi/3)*self.dy)

        # self.left = u1 #*36
        # self.center = u2 #*40
        # self.right = u3 #*40
        
        # self.right= -(cos(pi/3)*(self.dx) + cos(pi/3)*(self.dy)- (self.dr))
        # self.left = (sin(pi/3*(self.dy)) - sin(pi/3)*d_right)
        # self.center = (-1/(self.base_width/2))*(d_left+d_right+d_center)
        
        # self.right = (2*self.dx-(self.dr*self.base))/(2*self.wheelrad)
        # self.left = (2*self.dx+(self.dr*self.base))/(2*self.wheelrad)
        # self.center = 0
        # dx = (l + r) / 2
        # dr = (r - l) / w
        # self.right = 1.0 * self.dx + self.dr * self.w / 2 
        # self.left = 1.0 * self.dx - self.dr * self.w / 2
        # self.pub_lmotor.publish(self.left)
        # self.pub_rmotor.publish(self.right)
        # rospy.spin()

if __name__ == '__main__':
    """ main """
    TwistToVel()
    rospy.spin()