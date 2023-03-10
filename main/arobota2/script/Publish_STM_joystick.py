#!/usr/bin/env python

import serial
import rospy
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import String, Float32

ser = serial.Serial("/dev/ttyUSB_DEVICE2", 115200)
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
        # self._left_wheel_power = int(msg.x)
        # self._center_wheel_power = int(msg.y)
        # self._right_wheel_power= int(msg.z)
        self._left_wheel_power = -int(msg.z) 
        self._center_wheel_power = -int(msg.x)
        self._right_wheel_power= -int(msg.y)
        #power command   
        power_message = "M0"+"A"+str(self._left_wheel_power)+"B"+str(self._right_wheel_power)+"C"+str(self._center_wheel_power)+"\r\n"
        ser.write(bytes(power_message, 'utf-8'))
        rospy.loginfo(power_message)
        #velocity command
        #velocity_message = "M2"+"A"+str(int(self._right_wheel_power))+"B"+str(self._center_wheel_power)+"C"+str(self._left_wheel_power)+"\r\n"
        #ser.write(bytes(velocity_message, 'utf-8'))
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
