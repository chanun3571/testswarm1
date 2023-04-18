#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import rospy
from geometry_msgs.msg import PoseArray,Pose, Point, PoseStamped
from agent_util_multi import allPosi_allOrien
import math
from numpy_ros import to_numpy, to_message

class assign_centroid():
    def __init__(self):
        rospy.init_node('centroid') 
        rospy.loginfo("start centroid")
        self._agent_util = allPosi_allOrien()
        self.allPosition = self._agent_util.allPosition()
        self.pubpoint1 = rospy.Publisher('/robot1_formation_pos', Point, queue_size=1)
        self.pubpoint2 = rospy.Publisher('/robot2_formation_pos', Point, queue_size=1)
        self.pubpoint3 = rospy.Publisher('/robot3_formation_pos', Point, queue_size=1)
        self.pubcentroid = rospy.Publisher('/centroid', Point, queue_size=1)
        rospy.Subscriber('/swarm/move_base_simple/goal', PoseStamped, self.submit_centroid)
        self.centroid_pos = Point()


    def submit_centroid(self, msg):
        self.centroid_pos.x = msg.pose.position.x
        self.centroid_pos.y = msg.pose.position.y
        self.pubcentroid.publish(self.centroid_pos)
        rospy.loginfo(self.centroid_pos)

    def separate_pos(self, centroid_pos):
        self.centroid = to_numpy(centroid_pos)
        d1 = np.array([(-0.3)*math.cos(math.pi/6),(-0.3)*math.sin(math.pi/6),0])  
        d2 = np.array([0,0.3,0])
        d3 = np.array([0.3*math.cos(math.pi/6),-0.3*math.sin(math.pi/6),0])
        p_robot1 = self.centroid + d1
        p_robot2 = self.centroid + d2
        p_robot3 = self.centroid + d3
        self.pubpoint1.publish(to_message(Point, p_robot1))
        self.pubpoint2.publish(to_message(Point, p_robot2))
        self.pubpoint3.publish(to_message(Point, p_robot3))


    def spin(self):
        while not rospy.is_shutdown():
            # try:
            self.separate_pos(self.centroid_pos)
            rospy.Rate(20).sleep()
            # except:
            #     pass
           
if __name__=='__main__':
    try:
        agent = assign_centroid()
        agent.spin()
    except rospy.ROSInterruptException:
        pass







