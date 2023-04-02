#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from geometry_msgs.msg import PoseArray

class PoseCollector:
    def __init__(self):
        # ROS Initialize
        # Number of Agents = 3
        agentnum = 3
        rospy.Subscriber("/robot1/amcl_pose", PoseWithCovarianceStamped, self.allpose1_callback)
        rospy.Subscriber("/robot2/amcl_pose", PoseWithCovarianceStamped, self.allpose2_callback)
        rospy.Subscriber("/robot3/amcl_pose", PoseWithCovarianceStamped, self.allpose3_callback)
        self.pub_allPose = rospy.Publisher("/allPose", PoseArray, queue_size=1)
        self.pose=PoseArray()

    def allpose1_callback(self, msg):
        msg.pose.pose = self.pose[0]
        
    def allpose2_callback(self, msg):
        msg.pose.pose = self.pose[1]

    def allpose3_callback(self, msg):
        msg.pose.pose = self.pose[2]



    def spin(self):
        while not rospy.is_shutdown():
            self.pub_allPose.publish(self.pose)
            rospy.Rate(10).sleep()

if __name__ == "__main__":
    rospy.init_node("PoseCollector", anonymous=True)
    try:
        posecollector = PoseCollector()
        posecollector.spin()
    except rospy.ROSInterruptException:
        pass
