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
        self.pubcentroid = rospy.Publisher('/centroid', Point, queue_size=1)
        rospy.Subscriber('/swarm/move_base_simple/goal',PoseStamped,self.resubmitcallback1)
        # self.pubrobot1 = rospy.Publisher('/robot1/move_base_simple/goal',PoseStamped, queue_size=10)
        # self.pubrobot2 = rospy.Publisher('/robot2/move_base_simple/goal',PoseStamped, queue_size=10)
        # self.pubrobot3 = rospy.Publisher('/robot3/move_base_simple/goal',PoseStamped, queue_size=10)

    def find_centroid(self):
        self.centroid_pos = Point()
        self.x = (self.allPosition[0][0]+self.allPosition[1][0]+self.allPosition[2][0])/3
        self.y = (self.allPosition[0][1]+self.allPosition[1][1]+self.allPosition[2][1])/3
        self.centroid_pos.x = self.x
        self.centroid_pos.y = self.y  
        self.pubcentroid.publish(self.centroid_pos)
        self.centroid = to_numpy(self.centroid_pos)

    def separate_pos(self,x,y,centroid): #triangular
        d1 = np.array([(-0.3)*math.cos(math.pi/6),(-0.3)*math.sin(math.pi/6),0])  
        d2 = np.array([0,0.3,0])
        d3 = np.array([0.3*math.cos(math.pi/6),-0.3*math.sin(math.pi/6),0])
        if self.x!=0 and self.y!=0: #at initialize x=y=0
            p_robot1 = self.centroid + d1
            p_robot2 = self.centroid + d2
            p_robot3 = self.centroid + d3
            self.pubpoint1.publish(to_message(Point, p_robot1))
            self.pubpoint2.publish(to_message(Point, p_robot2))
            self.pubpoint3.publish(to_message(Point, p_robot3))

    def spin(self):
        while not rospy.is_shutdown():
            self.find_centroid()
            self.separate_pos(self.x,self.y,self.centroid)
            rospy.Rate(20).sleep()
           
if __name__=='__main__':
    try:
        agent=centroid()
        agent.spin()
    except rospy.ROSInterruptException:
        pass







