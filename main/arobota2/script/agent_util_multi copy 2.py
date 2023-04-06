#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import rospy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Empty, String, ColorRGBA


class AgentUtil(object):
    def __init__(self):
        self.agentNum = rospy.get_param("/agentNum")
        self.agentID = rospy.get_param("agentID")
        rospy.loginfo("starting node:agent" + str(self.agentID))

          # allPositions includes myself position
        self.allPositions = np.zeros((self.agentNum, 3))
        #add
        self.allOrientations = np.zeros((self.agentNum, 4))


        # subscriber to other agent's position
        rospy.Subscriber("/allPose", PoseArray, self.poseArrayCallback, queue_size=1)

        rospy.Publisher("/allPosition",)
        # subscriber to other agent:s orient  


    def poseArrayCallback(self, msg):
        # subscriber to get every agent's position
        arraynum = len(msg.poses)
        if (self.checkNeighborStart == False) and (arraynum == self.agentNum):
            # if the pose array contains every agent's pose, set the checkNeighborstart flag as True
            self.checkNeighborStart = True
            rospy.loginfo("NeighborStart")

        elif self.checkNeighborStart:
            for i in range(arraynum):
                pos = [
                    msg.poses[i].position.x,
                    msg.poses[i].position.y,
                    msg.poses[i].position.z,
                ]
                self.allPositions[i] = pos
    
    def orienArrayCallback(self, msg):
        # subscriber to get every agent's position
        arraynum = len(msg.poses)
        if (self.checkNeighborStart == False) and (arraynum == self.agentNum):
            # if the pose array contains every agent's pose, set the checkNeighborstart flag as True
            self.checkNeighborStart = True
            rospy.loginfo("NeighborStart")

        elif self.checkNeighborStart:
            for i in range(arraynum):
                orien = [
                    msg.orien[i].orientation.x,
                    msg.orien[i].orientation.y,
                    msg.orien[i].orientation.z,
                    msg.orien[i].orientation.w,
                ]
                self.allOrientations[i] = orien
    
    







