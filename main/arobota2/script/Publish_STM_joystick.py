#!/usr/bin/env python

import serial
import rospy
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import String, Float32

ser = serial.Serial("/dev/ttyUSB1", 115200) #Open port with baud rate 

class STM_Connect():
    def __init__(self): 
        rospy.init_node('STM_Pub',anonymous=True)
        self._left_wheel_power = 0
        self._right_wheel_power = 0
        self._center_wheel_power = 0
        #self._left_wheel_speed = 0
        #self._right_wheel_speed = 0
        #self._center_wheel_speed = 0
        rospy.Subscriber('/joystick', Vector3, self.joystickCallback)
        #rospy.loginfo("Publish data to STM")

    def joystickCallback(self, msg):
        r = rospy.Rate(20)
        #M0 = power command 
        #M1 = position command
        #M2 = velocity command
        self._left_wheel_power = msg.x * 10
        self._center_wheel_power = msg.y * 10
        self._right_wheel_power= msg.z * 10   
        #power command   
        speed_message = "M0"+"A"+str(int(self._right_wheel_power))+"B"+str(self._center_wheel_power)+"C"+str(self._left_wheel_power)+"\r\n"
        ser.write(bytes(speed_message, 'utf-8'))
        #rospy.loginfo(speed_message)
        r.sleep()
        #velocity command
        #velocity_message = "M2"+"A"+str(int(self._right_wheel_power))+"B"+str(self._center_wheel_power)+"C"+str(self._left_wheel_power)+"\r\n"
        #rospy.loginfo(velocity_message)

        
if __name__ =='__main__':
	try:
		STM_Connect() 	
		rospy.spin()
	except rospy.ROSInterruptException:
		rospy.logwarn("Connection Failed")
