#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from xbox_button import XBoxButton
from geometry_msgs.msg import Pose,Twist,PoseStamped,Vector3
from math import sin, cos, pi, sqrt
#from std_msgs.msg import String#String message 
#from std_msgs.msg import Int8
#from geometry_msgs.msg import Vector3


class Joystick_Input():
    def __init__(self):
        rospy.init_node('joystickinput1',anonymous=True)
        self._uh = Twist()
        self.pub = rospy.Publisher('joystick', Vector3, queue_size=10) 
        self.pubvel1 = rospy.Publisher('robot1/cmd_vel',Twist, queue_size=10)
        self.pubvel2 = rospy.Publisher('robot2/cmd_vel',Twist, queue_size=10)
        self.pubvel3 = rospy.Publisher('robot3/cmd_vel',Twist, queue_size=10)

        rospy.Subscriber('/joy',Joy,self.joy_callback) #joy

    def joy_callback(self, msg):
        joy_ux = msg.axes[XBoxButton.LX]
        joy_uy = msg.axes[XBoxButton.LY]
        joy_omega = msg.axes[XBoxButton.RX]
        self._uh.linear.x = joy_ux #(-1,1)
        self._uh.linear.y = joy_uy #(-1,1)
        self._uh.angular.z = joy_omega*8 #(-1,1)
        self.pubvel1.publish(self._uh)
        self.pubvel2.publish(self._uh)
        self.pubvel3.publish(self._uh)

    def motion(self,w,vx,vy):
        rate = rospy.Rate(10)
        #r = (48/2)/1000 # m
        r = 125/1000 #m
        #print(w,vx,vy)ss
        #u1 = 1/r*(-d*w + vx)
        #u2 = 1/r*(-d*w -cos(pi/3)*vx -sin(pi/3)*vy)
        #u3 = 1/r*(-d*w -cos(pi/ss3)*vx + sin(pi/3)*vy)
        #u1 = (-d*w + vx)
        #u2 = (-d*w -cos(pi/3)*vx -sin(pi/3)*vy)
        #u3 = (-d*w -cos(pi/3)*vx + sin(pi/3)*vy)
        v_center = ((1/3)*r*w - (2/3)*vy)
        v_right = ((1/3)*r*w +(sqrt(3)/3)*vx + (1/3)*vy)
        v_left = ((1/3)*r*w -(sqrt(3)/3)*vx + (1/3)*vy)
        # v_center = (r*w - vy)
        # v_right = (r*w +(1/cos(pi/6))*vx +(1/sin(pi/6))*vy)
        # v_left = (r*w -(1/cos(pi/6))*vx + (1/sin(pi/6))*vy)
        ros_translation = Vector3()
        ros_translation.x = v_center *40
        ros_translation.y = v_right *40
        ros_translation.z = v_left *40
        self.pub.publish(ros_translation)
        #rospy.loginfo(ros_translation)
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
