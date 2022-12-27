#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from xbox_button import XBoxButton
from geometry_msgs.msg import Pose,Twist,PoseStamped,Vector3
from math import sin, cos, pi
#rom std_msgs.msg import String#String message 
#from std_msgs.msg import Int8
#from geometry_msgs.msg import Vector3


class Joystick_Input():
    def __init__(self):
        rospy.init_node('joystickinput1',anonymous=True)
        self._uh = Twist()
        self.pub = rospy.Publisher('joystick', Vector3, queue_size=10000) 
        rospy.Subscriber('/joy',Joy,self.joy_callback) #joy

    def joy_callback(self, msg):
        joy_ux = msg.axes[XBoxButton.LX]
        joy_uy = msg.axes[XBoxButton.LY]
        joy_omega = msg.axes[XBoxButton.RX]
        self._uh.linear.x = joy_ux #(-1,1)
        self._uh.linear.y = joy_uy #(-1,1)
        self._uh.angular.z = joy_omega #(-1,1)

    def motion(self,w,vx,vy):
        rate = rospy.Rate(10)
        #r = (48/2)/1000 # m
        d = 200/1000 #m
        #print(w,vx,vy)
        #u1 = 1/r*(-d*w + vx)
        #u2 = 1/r*(-d*w -cos(pi/3)*vx -sin(pi/3)*vy)
        #u3 = 1/r*(-d*w -cos(pi/3)*vx + sin(pi/3)*vy)
        u1 = (-d*w + vx)
        u2 = (-d*w -cos(pi/3)*vx -sin(pi/3)*vy)
        u3 = (-d*w -cos(pi/3)*vx + sin(pi/3)*vy)
        ros_translation = Vector3()
        ros_translation.x = u1 
        ros_translation.y = u2 
        ros_translation.z = u3 
        self.pub.publish(ros_translation)
        rospy.loginfo(ros_translation)
        rate.sleep()

    def spin(self):
        # initialize message
        while not rospy.is_shutdown():
            #rate = rospy.Rate(100)
            self.motion(self._uh.angular.z,self._uh.linear.x,self._uh.linear.y)
            #rate.sleep()
            

if __name__=='__main__':
    try:
        agent=Joystick_Input()
        agent.spin()
    except rospy.ROSInterruptException:
        pass
