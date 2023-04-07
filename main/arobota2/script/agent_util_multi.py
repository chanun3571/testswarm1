#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import rospy
from geometry_msgs.msg import PoseArray
from numpy_ros import to_numpy, to_message

class allPosi_allOrien():
    def __init__(self):
        # rospy.init_node('allPosi_allOrien') 
        # subscriber to all pose
        rospy.Subscriber('/allpose', PoseArray, self.poseArrayCallback)
        self.allPositions = np.zeros((3, 3))
        self.allOrientations = np.zeros((3, 4))
    def poseArrayCallback(self, msg):
        # subscriber to get every agent's position
        arraynum = len(msg.poses)
        # rospy.loginfo(arraynum)
        for i in range(arraynum):
            pos = [
                msg.poses[i].position.x,
                msg.poses[i].position.y,
                msg.poses[i].position.z,
            ]
            orien = [
                msg.poses[i].orientation.x,
                msg.poses[i].orientation.y,
                msg.poses[i].orientation.z,
                msg.poses[i].orientation.w,
            ]
            self.allPositions[i] = pos
            self.allOrientations[i] = orien


    def allPosition(self):
        return self.allPositions
    def allOrientation(self):
        return self.allOrientations

    
#     def spin(self):
#         if ready =
           
# if __name__=='__main__':
#     try:
#         agent=allPosi_allOrien()
#         agent.allPosition()
#     except rospy.ROSInterruptException:
#         pass







