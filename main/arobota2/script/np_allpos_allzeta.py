#!/usr/bin/env python

import tf
import numpy as np
import rospy

from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

class np_allpose:
    def __init__(self):
        rospy.init_node("np_allpose", anonymous=True)
        # self.pub_zetas = rospy.Publisher("zetas", Float32MultiArray, queue_size=1)
        # self.pub_positions = rospy.Publisher("positions", Float32MultiArray, queue_size=1)
        rospy.Subscriber("/allpose", PoseArray, self.poseArrayCallback, queue_size=1)
    def poseArrayCallback(self, msg):
        arraynum = len(msg.poses)
        self.positions = np.zeros((3,3))
        self.zetas = np.zeros((3,1))

        for i in range(arraynum):
            pos = [
                msg.poses[i].position.x,
                msg.poses[i].position.y,
                msg.poses[i].position.z,
                ]
            quat = [
                msg.poses[i].orientation.x,
                msg.poses[i].orientation.y,
                msg.poses[i].orientation.z,
                msg.poses[i].orientation.w
                ]
            quat = np.array(quat)
            angle = tf.transformations.euler_from_quaternion(quat)
            angle = angle[2]  # angle about the z-axis
            self.positions[i]=pos
            self.zetas[i]= angle
        my_msg = Float32MultiArray()
        my_msg.data= self.positions
        # print(type(self.positions))
        # print(self.zetas)
        # self.positions = Float32MultiArray(data=self.positions)
        # self.positions = Float32MultiArray(data=self.zetas)

        # self.pub_zetas.publish(self.zetas)
        self.pub_positions.publish(my_msg)   

if __name__=='__main__':
    try:
        np_allpose()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
