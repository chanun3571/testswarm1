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

        # param initialize
        # allPositions includes myself position
        self.allPositions = np.zeros((self.agentNum, 3))
        #add
        self.allOrientations = np.zeros((self.agentNum, 4))

        # publisher for agent control
        self.pub_twist = rospy.Publisher("cmd_input", Twist, queue_size=1)
        self.pub_takeoffland = rospy.Publisher("cmd_takeoffland", String, queue_size=1)

        self.checkNeighborStart  = False

        # subscriber to get own pose
        rospy.Subscriber("posestamped", PoseStamped, self.poseStampedCallback, queue_size=1)

        # subscriber to other agent's position
        rospy.Subscriber("/allPose", PoseArray, self.poseArrayCallback, queue_size=1)
        # subscriber to other agent:s orient
        rospy.Subscriber("/allOrien", PoseArray, self.orienArrayCallback, queue_size=1)
        rospy.wait_for_message("posestamped", PoseStamped)

    
    ###################################################################
    ### public functions
    ###################################################################

    def get_agent_num(self):
        return self.agentNum
    
    def get_my_id(self):
        return self.agentID
    
    def get_my_pose(self):
        return self.position, self.orientation
    
    def get_all_positions(self):
        return self.allPositions

    #new function
    
    def get_all_orientations(self):
        return self.allOrientations
    
    def get_neighbor_positions(self):
        neighbor_position = np.delete(self.allPositions, self.agentID - 1, axis=0)
        return neighbor_position
    
    def is_neighbor_started(self):
        return self.checkNeighborStart

    def publish_takeoff(self):
        pubString = String("takeoff")
        self.pub_takeoffland.publish(pubString)
    
    def publish_land(self):
        pubString = String("land")
        self.pub_takeoffland.publish(pubString)
    
    def stop_takeoff(self):
        pubString = String('')
        self.pub_takeoffland.publish(pubString)
    
    
    def publish_command(self, ux, uy, uz, omega_z):
        twist = Twist()
        twist.linear.x = ux
        twist.linear.y = uy
        twist.linear.z = uz
        twist.angular.z = omega_z
        self.pub_twist.publish(twist)


    ###################################################################
    ### subscriber callback functions
    ###################################################################

    def poseStampedCallback(self, pose_msg):
        # subscriber to get own pose(position)
        self.position = np.array(
            [
                pose_msg.pose.position.x,
                pose_msg.pose.position.y,
                pose_msg.pose.position.z,
            ]
        )
        self.orientation = np.array(
            [
                pose_msg.pose.orientation.x,
                pose_msg.pose.orientation.y,
                pose_msg.pose.orientation.z,
                pose_msg.pose.orientation.w,
            ]
        )

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
    
    







