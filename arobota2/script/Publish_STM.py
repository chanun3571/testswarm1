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
        self.wheel_speed = []
        self.wheel = 0

        rospy.loginfo("Publish data to STM")

        # self._left_wheel_speed = rospy.Subscriber('lwheel_vtarget',Float32,self.Update_Left_Speed)
        # self._right_wheel_speed = rospy.Subscriber('rwheel_vtarget',Float32,self.Update_Right_Speed)
        self.wheel_speed = rospy.Subscriber('wheel_vtarget',String,self.Update_Speed)

    # def Update_Left_Speed(self, left_speed):
        
    #     self._left_wheel_speed_ = left_speed.data
    #     rospy.loginfo(left_speed.data)
    #     speed_message = "M2A"+str(int(self._left_wheel_speed_*100000000))+"\r\n"
    #     print(speed_message)

    #     ser.write(bytes(speed_message, 'utf-8'))

    # def Update_Right_Speed(self, right_speed):
    #     self._left_wheel_speed_ = right_speed.data
    #     rospy.loginfo(right_speed.data)
    #     speed_message = "M2B"+str(int(self._right_wheel_speed_*100000000))+"\r\n"
    #     print(speed_message)

    #     ser.write(bytes(speed_message, 'utf-8'))

    def Update_Speed(self, msg):
        self.wheel = msg.data.split(',')
        # print ("M2A"+str(int(float(self.wheel[0])*1000000))+"B"+str(int(float(self.wheel[1])*1000000))+"\r\n")
        # rospy.loginfo(msg.data)
        speed_message = "M2A"+str(int(float(self.wheel[0])*5000))+"B"+str(int(float(self.wheel[1])*5000))+"\r\n"
        print(speed_message)

        ser.write(bytes(speed_message, 'utf-8'))
        

if __name__ =='__main__':
	
	try:
		STM_Connect() 	
		rospy.spin()
	except rospy.ROSInterruptException:
		rospy.logwarn("Connection Failed")
