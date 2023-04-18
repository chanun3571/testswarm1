#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import rospy
from geometry_msgs.msg import PoseArray,Pose, Point, PoseStamped, Pose, Quaternion
from agent_util_multi import allPosi_allOrien
import math
from numpy_ros import to_numpy, to_message

class assign_centroid():
    def __init__(self):
        rospy.init_node('centroid') 
        rospy.loginfo("start centroid")
        self._agent_util = allPosi_allOrien()
        self.allPosition = self._agent_util.allPosition()
        self.pubpoint1 = rospy.Publisher('/robot1_formation_pos', Pose, queue_size=1)
        self.pubpoint2 = rospy.Publisher('/robot2_formation_pos', Pose, queue_size=1)
        self.pubpoint3 = rospy.Publisher('/robot3_formation_pos', Pose, queue_size=1)
        self.pubcentroid = rospy.Publisher('/centroid', Pose, queue_size=1)
        rospy.Subscriber('/swarm/move_base_simple/goal', PoseStamped, self.submit_centroid)

    def submit_centroid(self, msg):
        self.centroid_pose = Pose()
        self.centroid_pose.position.x = msg.pose.position.x
        self.centroid_pose.position.y = msg.pose.position.y
        self.centroid_pose.orientation.z = msg.pose.orientation.z
        self.centroid_pose.orientation.w = msg.pose.orientation.w
        self.pubcentroid.publish(self.centroid_pose)
        rospy.loginfo(self.centroid_pose)

        self.centroid = to_numpy(self.centroid_pose.position)
        d1 = np.array([(-0.3)*math.cos(math.pi/6),(-0.3)*math.sin(math.pi/6),0])  
        d2 = np.array([0,0.3,0])
        d3 = np.array([0.3*math.cos(math.pi/6),-0.3*math.sin(math.pi/6),0])
        p_robot1 = to_message(Point,self.centroid + d1)
        p_robot2 = to_message(Point,self.centroid + d2)
        p_robot3 = to_message(Point,self.centroid + d3)
        
        self.pubpoint1.publish(Pose(p_robot1, self.centroid_pose.orientation))
        self.pubpoint2.publish(Pose(p_robot2, self.centroid_pose.orientation))
        self.pubpoint3.publish(Pose(p_robot3, self.centroid_pose.orientation))
        # self.pubpoint1.publish(to_message(Pose, p_robot1))
        # self.pubpoint2.publish(to_message(Pose, p_robot2))
        # self.pubpoint3.publish(to_message(Pose, p_robot3))
           
if __name__=='__main__':
    try:
        agent = assign_centroid()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass







