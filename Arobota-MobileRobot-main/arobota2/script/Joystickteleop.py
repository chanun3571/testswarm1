#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from xbox_button import XBoxButton
from geometry_msgs.msg import Pose,Twist,PoseStamped,Vector3

#rom std_msgs.msg import String#String message 
#from std_msgs.msg import Int8
#from geometry_msgs.msg import Vector3

pub = rospy.Publisher('joystick', Vector3, queue_size=1) # "key" is the publisher name

class Joystick_Input():
    def __init__(self):
        rospy.init_node('joystickinput1',anonymous=True)
        self._uh = Twist()
        self._pub_uh = rospy.Publisher('uh',Twist, queue_size=1)
        self.x = 0
        self.y = 0
        self.omega = 0 
        self.button = ""
        rospy.Subscriber('/joy',Joy,self.joy_callback) #joy

    def joy_callback(self, msg):
        joy_ux = msg.axes[XBoxButton.LX]
        joy_uy = msg.axes[XBoxButton.LY]
        joy_omega = msg.axes[XBoxButton.RX]
        self._uh.linear.x = joy_ux #(-1,1)
        self._uh.linear.y = joy_uy #(-1,1)
        self._uh.angular.z = joy_omega #(-1,1)

    def motion(self,w,vx,vy):
        r = (45/2)/100 #m
        d = 60/100 #m
        #print(w,vx,vy)
        u1 = 1/r*(-d*w + vx)
        u2 = 1/r*(-d*w -1/2*vx -0.866*vy)
        u3 = 1/r*(-d*w -1/2*vx +0.866*vy)
        ros_translation = Vector3()
        ros_translation.x = u1
        ros_translation.y = u2
        ros_translation.z = u3
        pub.publish(ros_translation)
        #rospy.loginfo(ros_translation)

    def spin(self):
        r = rospy.Rate(100)
        # initialize message
        while not rospy.is_shutdown():
            self.motion(self._uh.angular.z,self._uh.linear.x,self._uh.linear.y)
            r.sleep()
            

if __name__=='__main__':
    try:
        agent=Joystick_Input()
        agent.spin()
    except rospy.ROSInterruptException:
        pass
