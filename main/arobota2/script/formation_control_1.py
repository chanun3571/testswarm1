#!/usr/bin/env python

from xbox_button import XBoxButton
import tf
import math
import numpy as np
import rospy

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Twist

class AgentManagerExample:
    def __init__(self):
        rospy.init_node("robot1_formation_velocity")
        self.myid = 1
        self.rate = rospy.Rate(10)
        self._uh = Twist()
        self.finalvelo= Twist()
        # self._my_zeta = np.zeros((1,3))
        # self._zetas = np.zeros((3,3))

        self._pub_zeta = rospy.Publisher("zeta", PoseStamped, queue_size=1)
        # rospy.Subscriber("/allPose", PoseArray, self.positionArrayCallback, queue_size=1)
        # rospy.Subscriber("/allZeta", PoseArray, self.zetaArrayCallback, queue_size=1)
        # rospy.Subscriber("/joy", Joy, self.joy_callback, queue_size=1)
        rospy.Subscriber("/allpose", PoseArray, self.poseArrayCallback, queue_size=1)
        self.pubvel1 = rospy.Publisher('robot1/cmd_vel',Twist, queue_size=10)
        self.pubvel2 = rospy.Publisher('robot2/cmd_vel',Twist, queue_size=10)
        self.pubvel3 = rospy.Publisher('robot3/cmd_vel',Twist, queue_size=10)
        rospy.Subscriber('/joy',Joy,self.joy_callback) #joy

    def joycallback(self,msg):
        joy_ux = msg.axes[XBoxButton.LX]
        joy_uy = msg.axes[XBoxButton.LY]
        joy_omega = msg.axes[XBoxButton.RX]
        self._uh.linear.x = joy_ux #(-1,1)
        self._uh.linear.y = joy_uy #(-1,1)
        self._uh.angular.z = joy_omega*8 #(-1,1)

    def poseArrayCallback(self, msg):
        arraynum = len(msg.poses)
        self.positions = np.zeros((3,3))
        self.zetas = np.zeros((3,1))

        for i in range(arraynum):
            pos = [
                msg.poses[i].position.x,
                msg.poses[i].position.y,
                msg.poses[i].position.z,
                ]
            quat = [
                msg.poses[i].orientation.x,
                msg.poses[i].orientation.y,
                msg.poses[i].orientation.z,
                msg.poses[i].orientation.w
                ]
            quat = np.array(quat)
            angle = tf.transformations.euler_from_quaternion(quat)
            angle = angle[2]  # angle about the z-axis
            self.positions[i]=pos
            self.zetas[i]= angle

    def main_control(self):
        ### you can use drone's position to design the input ###
        all_positions = self.positions

        #triangular    
        d1 = np.array([(-0.25)*math.cos(math.pi/6),(-0.25)*math.sin(math.pi/6),0])  
        d2 = np.array([0,0.25,0])
        d3 = np.array([0.25*math.cos(math.pi/6),-0.25*math.sin(math.pi/6),0])
        q1= all_positions[0] + d1
        q2= all_positions[1] + d2
        q3= all_positions[2] + d3
        
        #faceoutside
        # zeta1 = self.zetas[0] #0 degree
        # zeta2 = self.zetas[1] + ((2*math.pi)/3) #120 degree
        # zeta3 = self.zetas[2] + ((4*math.pi)        self.myid = 1/3) #360 degree
        
        if self.myid==1:
            self.velocity = (q2-q1) + (q3-q1) 
        if self.myid==2:
            self.velocity = (q1-q2) + (q3-q2) 
        if self.myid==3:
            self.velocity = (q1-q3) + (q2-q3)

        self.velocity_x, self.velocity_y = self.velocity[0]+self._uh.linear.x,self.velocity[1]+self._uh.linear.y
        
        self.finalvelo.linear.x = self.velocity_x
        self.finalvelo.linear.x = self.velocity_y
        self.finalvelo.linear.x = self._uh.angular.z,
        if self.myid==1:
            self.pubvel1.publish(self.finalvelo)
        if self.myid==2:
            self.pubvel2.publish(self.finalvelo)
        if self.myid==3:
            self.pubvel3.publish(self.finalvelo)       

    def spin(self):
        # initialize message
        while not rospy.is_shutdown():
            self.main_control()

if __name__=='__main__':
    try:
        agent = AgentManagerExample()
        agent.spin()
    except rospy.ROSInterruptException:
        pass