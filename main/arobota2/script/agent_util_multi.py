#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import rospy
from geometry_msgs.msg import PoseArray



class allPosi_allOrien():
    def __init__(self):
        rospy.init_node('allPosi_allOrien') 
        self.allPositions = np.zeros((3, 3))
        self.allOrientations = np.zeros((3, 4))

        # subscriber to all pose
        rospy.Subscriber('/allpose', PoseArray, self.poseArrayCallback)



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
        rospy.loginfo((self.allOrientations))
        self.allPosi.publish(self.allPositions)
        self.allOrien.publish(self.allOrientations)
            
if __name__=='__main__':
    try:
        allPosi_allOrien()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass







