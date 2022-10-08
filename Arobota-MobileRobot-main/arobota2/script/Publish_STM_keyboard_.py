#!/usr/bin/env python3
import serial
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32

ser = serial.Serial ("/dev/ttyUSB1", 115200) #Open port with baud rate 
class STM_Connect():
    def __init__(self): 
        rospy.init_node('STM_Pub',anonymous=True)
        self._left_wheel_speed_ = 0
        self._right_wheel_speed_ = 0
        self._center_wheel_speed_ = 0
        self.wheel_speed = []
        self.wheel = 0
        rospy.loginfo("Publish data to STM")

    def Update_Speed(self, msg):
        self.wheel = msg.data.split(',')
        #M0 = power command 
        #M1 = position command
        #M2 = velocity command
        u_x =
        u_y = 
        speed_message = "M2"+"A"+str(int(float(self.wheel[0])*5000))+"B"+str(int(float(self.wheel[1])*5000))+"C"+str(int(float(self.wheel[2])*5000))+"\r\n"
        print(speed_message)
        ser.write(bytes(speed_message, 'utf-8'))
        
if __name__ =='__main__':
	try:
		STM_Connect() 	
		rospy.spin()
	except rospy.ROSInterruptException:
		rospy.logwarn("Connection Failed")
