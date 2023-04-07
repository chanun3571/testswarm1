#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import rospy
from geometry_msgs.msg import PoseArray,Pose, Point
from agent_util_multi import allPosi_allOrien
import math
from numpy_ros import to_numpy, to_message

class centroid():
    def __init__(self):
        rospy.init_node('centroid') 
        rospy.loginfo("start centroid")
        self._agent_util = allPosi_allOrien()
        self.allPosition = self._agent_util.allPosition()
        self.pubpoint1 = rospy.Publisher('/robot1_formation_pos', Point, queue_size=1)
        self.pubpoint2 = rospy.Publisher('/robot2_formation_pos', Point, queue_size=1)
        self.pubpoint3 = rospy.Publisher('/robot3_formation_pos', Point, queue_size=1)


    def find_centroid(self):
        self.x = self.allPosition[0][0]+self.allPosition[1][0]+self.allPosition[2][0]
        self.y = self.allPosition[0][1]+self.allPosition[1][1]+self.allPosition[2][1]
        # print(self.allPosition)

    def separate_pos(self,x,y):
        d1 = np.array([(-1)*math.cos(math.pi/6),(-1)*math.sin(math.pi/6),0])  
        d2 = np.array([0,1,0])
        d3 = np.array([1*math.cos(math.pi/6),-1*math.sin(math.pi/6),0])
        if self.x!=0 and self.y!=0: #at initialize x=y=0
            p_robot1 = self.allPosition[0] + d1
            p_robot2 = self.allPosition[1] + d2
            p_robot3 = self.allPosition[2] + d3
            self.pubpoint1.publish(to_message(Point, p_robot1))
            self.pubpoint2.publish(to_message(Point, p_robot2))
            self.pubpoint3.publish(to_message(Point, p_robot3))


    def spin(self):
        while not rospy.is_shutdown():
            self.find_centroid()
            self.separate_pos(self.x,self.y)
            rospy.Rate(20).sleep()


           
if __name__=='__main__':
    try:
        agent=centroid()
        agent.spin()
    except rospy.ROSInterruptException:
        pass







