#!/usr/bin/env python3
import serial
from yaml import StreamEndEvent
import rospy
import threading
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16,Int32, Int64, Float32, String, Header, UInt64

# while True:
#     received_data = ser.read()              #read serial port
#     sleep(0.03)
#     data_left = ser.inWaiting()             #check for remaining byte
#     received_data += ser.read(data_left)
#     print (received_data)                   #print received data
#     ser.write(received_data)    

ser = serial.Serial ("/dev/ttyUSB_DEVICE2", 115200)
class STM_Connect():
    def __init__(self): 
        rospy.init_node('STM_Sub',anonymous=True)
        self._left_encoder_value = 0
        self._right_encoder_value = 0
        self._center_encoder_value = 0
        rospy.loginfo("Receive Data from STM")
        streamEncoder()
        # serialReadThread = threading.Thread(target=receiveSerial)
        # serialReadThread.start()
        
        
        #Publisher for left and right wheel encoder values
        self._Left_Encoder = rospy.Publisher('lwheel',Int64,queue_size=10)		
        self._Right_Encoder = rospy.Publisher('rwheel',Int64,queue_size=10)
        self._Center_Encoder = rospy.Publisher('cwheel',Int64,queue_size=10)

        # #Publisher for entire serial data
        # self._SerialPublisher = rospy.Publisher('serial', String,queue_size=10) 


    def Update_recieve_data(self):

        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if ser.in_waiting > 0:
                try:
                    recv = ser.read(ser.in_waiting)
                    msg = recv.decode('ascii')
                    print(msg)
                    #rospy.loginfo(msg)
                    self.encoder_value = msg.split(", ")
                    #rospy.loginfo(self.encoder_value)

                    self.left_encoder_value = (self.encoder_value[0].split("="))
                    self.left_encoder_value = int(self.left_encoder_value[1])

                    self.right_encoder_value = int(self.encoder_value[1]) #ok

                    self.center_encoder_value = (self.encoder_value[2].split("\r"))
                    self.center_encoder_value = int(self.center_encoder_value[0])

                    self._Left_Encoder.publish(int(self.left_encoder_value))
                    self._Right_Encoder.publish(int(self.right_encoder_value))
                    self._Center_Encoder.publish(int(self.center_encoder_value))
                except:
                    continue
            rate.sleep()

def sendSerial(string):
    packet = bytes(string+"\r\n",'ascii')
    ser.write(packet)

def streamEncoder():
    sendSerial("R")
    sendSerial("S1")


if __name__ =='__main__':
	try:
		con = STM_Connect() 	
		con.Update_recieve_data()
	except rospy.ROSInterruptException:
		rospy.logwarn("Connection Failed")
