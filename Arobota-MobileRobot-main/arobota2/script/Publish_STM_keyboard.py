#!/usr/bin/env python3
import serial
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32

ser = serial.Serial ("/dev/ttyUSB1", 115200) #Open port with baud rate 

class STM_Connect():
    def __init__(self): 
        rospy.init_node('STM_Pub',anonymous=True)
        self._left_wheel_power = 0
        self._right_wheel_power = 0
        self._center_wheel_power = 0
        self._left_wheel_speed = 0
        self._right_wheel_speed = 0
        self._center_wheel_speed = 0
        rospy.Subscriber('/keyboard', String, self.keyboardCallback)
        rospy.loginfo("Publish data to STM")

    def keyboardCallback(self, msg):
        #M0 = power command 
        #M1 = position command
        #M2 = velocity command
        #power command 
        #keyboard can't input value between 0 and 1 + only one axis (one button) at a time
        self._left_wheel_power = msg.x * 10
        self._center_wheel_power = msg.y * 10
        self._right_wheel_power= msg.z * 10   
        #power command   
        speed_message = "M0"+"A"+str(int(self._left_wheel_power))+"B"+str(self._center_wheel_power)+"C"+str(self._right_wheel_power)+"\r\n"
        ser.write(bytes(speed_message, 'utf-8'))

    def spin(self):
        
        
if __name__ =='__main__':
	try:
		STM_Connect() 	
		rospy.spin()
	except rospy.ROSInterruptException:
		rospy.logwarn("Connection Failed")
