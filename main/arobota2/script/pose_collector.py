#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from geometry_msgs.msg import PoseArray

class Collector:
    def __init__(self, agentName):
        self.ready = False
        topicName = rospy.get_param("~posestampedTopic")
        preTopicName = rospy.get_param("~preTopicName", "/")
        subTopic = preTopicName + agentName + topicName
        rospy.loginfo("topicName:" + subTopic)
        # subscriber for each agent's region
        rospy.Subscriber(subTopic, PoseStamped, self.poseStampedCallback, queue_size=1)
        # initialze with zeros
        self.pose = Pose()

    def poseStampedCallback(self, msg_data):
        if self.ready == False:
            self.ready = True
        self.pose = msg_data.pose

    def getPose(self):
        return self.pose

    def getReady(self):
        return self.ready


class PoseCollector:
    def __init__(self):
        # ROS Initialize
        # Number of Agents = 3
        rospy.Subscriber("/robot1/amcl_pose", PoseWithCovarianceStamped, self.allpose1_callback)
        rospy.Subscriber("/robot2/amcl_pose", PoseWithCovarianceStamped, self.allpose2_callback)
        rospy.Subscriber("/robot3/amcl_pose", PoseWithCovarianceStamped, self.allpose3_callback)
        self.pub_allPosition = rospy.Publisher("/allPos", PoseArray, queue_size=1)
        self.pub_allOrientation = rospy.Publisher("/allOrien", PoseArray, queue_size=1)

        self.CollectorPosi = [[0,0],[0,0],[0,0]]
        self.CollectorOrien =[[0,0],[0,0],[0,0]]
        
        def allpose1_calllback(self, msg):
            msg.pose.pose.position.x=self.CollectorPosi[0][0]
            msg.pose.pose.position.y=self.CollectorPosi[0][1]
            msg.pose.pose.orientation.z=self.CollectorOrien[0][0]
            msg.pose.pose.orientation.w=self.CollectorOrien[0][1]
        def allpose2_calllback(self, msg):
            msg.pose.pose.position.x=self.CollectorPosi[1][0]
            msg.pose.pose.position.y=self.CollectorPosi[1][1]
            msg.pose.pose.orientation.z=self.CollectorOrien[1][0]
            msg.pose.pose.orientation.w=self.CollectorOrien[1][1]
        def allpose2_calllback(self, msg):
            msg.pose.pose.position.x=self.CollectorPosi[2][0]
            msg.pose.pose.position.y=self.CollectorPosi[2][1]
            msg.pose.pose.orientation.z=self.CollectorOrien[2][0]
            msg.pose.pose.orientation.w=self.CollectorOrien[2][1]

    def spin(self):
        while not rospy.is_shutdown():
            self.pub_allPosition.publish(self.CollectorPosi)
            self.pub_allOrientation.publish(self.CollectorOrien)
            self.CollectorPosi = [[0,0],[0,0],[0,0]]
            self.CollectorOrien =[[0,0],[0,0],[0,0]]
            self.Rate(10).sleep()

if __name__ == "__main__":
    rospy.init_node("poseCollector", anonymous=True)
    try:
        posecollector = PoseCollector()
        posecollector.spin()
    except rospy.ROSInterruptException:
        pass
