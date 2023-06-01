#!/usr/bin/env python
# -*- coding: utf-8 -*-
from bebop_hatanaka_base.agent_base import AgentBase

import tf
import rospy
from geometry_msgs.msg import Pose, PoseStamped, PoseArray


import numpy as np


class PublishCentralPosition:
    def __init__(self):
        agents_param = rospy.get_param("/agents")
        self._clock = agents_param["agent_manager_clock"]

        self._agent_base = AgentBase()

        self._pub_pose = rospy.Publisher("/centralpose", PoseStamped, queue_size=1)
        self._pub_pose_euler = rospy.Publisher(
            "/centralposeeuler", PoseStamped, queue_size=1
        )
        rospy.wait_for_message("/allPose", PoseArray)

    ###################################################################
    ### main
    ###################################################################
    def main_control(self):
        all_positions = self._agent_base.get_all_positions()
        all_orientations = self._agent_base.get_all_orientations()
        p1 = all_positions[0]
        p2 = all_positions[1]

        # drone1
        quat1 = np.array(all_orientations[0])
        angle1 = tf.transformations.euler_from_quaternion(quat1)
        angle1 = angle1[2]  # angle about the z-axis

        # drone2
        quat2 = np.array(all_orientations[1])
        angle2 = tf.transformations.euler_from_quaternion(quat2)
        angle2 = angle2[2]  # angle about the z-axis

        # average position of accessible drones (#101 and #102) --> why not of all drones? --> as drone #4
        y_h_pos = (p1 + p2) / 2  # position101+position102/2
        y_h_omega = (angle1 + angle2) / 2  # orientation101+orientation102/2

        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "center"

        msg.pose.position.x = y_h_pos[0]
        msg.pose.position.y = y_h_pos[1]
        msg.pose.position.z = y_h_pos[2]
        values = tf.transformations.quaternion_from_euler(0, 0, y_h_omega)
        msg.pose.orientation.x = values[0]
        msg.pose.orientation.y = values[1]
        msg.pose.orientation.z = values[2]
        msg.pose.orientation.w = values[3]
        self._pub_pose.publish(msg)

        msg1 = PoseStamped()
        msg1.header.stamp = rospy.Time.now()
        msg1.header.frame_id = "center_euler"
        msg1.pose.position.x = y_h_pos[0]
        msg1.pose.position.y = y_h_pos[1]
        msg1.pose.position.z = y_h_pos[2]
        msg1.pose.orientation.z = y_h_omega
        self._pub_pose_euler.publish(msg1)

    ###################################################################
    ### spin
    ###################################################################
    def spin(self):
        rate = rospy.Rate(self._clock)
        while not rospy.is_shutdown():
            self.main_control()
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("agent", anonymous=True)
    agent = PublishCentralPosition()
    agent.spin()