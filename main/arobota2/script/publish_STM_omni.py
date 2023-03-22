#!/usr/bin/env python3
import serial
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32

ser = serial.Serial ("/dev/ttyUSB_DEVICE2", 115200) #Open port with baud rate 
class STM_Connect():
    def __init__(self): 
        rospy.init_node('STM_Pub',anonymous=True)
        self.left_wheel_speed_ = 0
        self.right_wheel_speed_ = 0
        self.center_wheel_speed_ = 0     
        rospy.loginfo("Publish data to STM")
        rospy.Subscriber('wheel_vtarget',String,self.Update_Speed)
        
    def Update_Speed(self, msg):
        r = rospy.Rate(10)
        self.wheel = msg.data.split(',')
        self.left_wheel_power = -int(float(self.wheel[0])*100)
        self.center_wheel_power = -int(float(self.wheel[1])*100)
        self.right_wheel_power = -int(float(self.wheel[2])*100)
        #dead zone
        # if self.left_wheel_power==0 and self.right_wheel_power==0 and self.center_wheel_power==0:
        #     print("no motion")
        # else:
        #     if 0<abs(self.left_wheel_power)<10 and 0<abs(self.right_wheel_power)<10 and 0<abs(self.center_wheel_power)<10:
        #         if self.left_wheel_power<0:
        #             self.left_wheel_power = self.left_wheel_power - 10
        #         if self.left_wheel_power<0:
        #             self.right_wheel_power = self.right_wheel_power -10 
        #         if self.center_wheel_power<0:
        #             self.center_wheel_power = self.center_wheel_power -10 
        #         if self.left_wheel_power>0:
        #             self.left_wheel_power = self.left_wheel_power + 10
        #         if self.left_wheel_power>0:
        #             self.right_wheel_power = self.right_wheel_power +10 
        #         if self.center_wheel_power>0:
        #             self.center_wheel_power = self.center_wheel_power +10 
               
        # print ("M2A"+str(int(float(self.wheel[0])*1000000))+"B"+str(int(float(self.wheel[1])*1000000))+"\r\n")
        # rospy.loginfo(msg.data)
        #power_message = "M0"+"A"+str(int(self._right_wheel_power)/500)+"B"+str(int(self._center_wheel_power)/500)+"C"+str(int(self._left_wheel_power)/500)+"\r\n"
        # self._left_wheel_power = -int(msg.z) 
        # self._center_wheel_power = -int(msg.x)
        # self._right_wheel_power= -int(msg.y)
        #power command   
        power_message = "M0"+"A"+str(self.left_wheel_power)+"B"+str(self.right_wheel_power)+"C"+str(self.center_wheel_power)+"\r\n"
        ser.write(bytes(power_message, 'utf-8'))
        rospy.loginfo(power_message)
        r.sleep()
        
if __name__ =='__main__':
	try:
		STM_Connect() 	
		rospy.spin()
	except rospy.ROSInterruptException:
		rospy.logwarn("Connection Failed")