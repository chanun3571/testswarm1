#!/usr/bin/env python

from xbox_button import XBoxButton
from agent_util import AgentUtil
from transformations import quaternion_matrix 

import math
import numpy as np
import rospy

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Pose, PoseStamped
from geometry_msgs.msg import PoseArray


class AgentManagerExample:
    def __init__(self):
        self.clock = rospy.get_param("/agent_manager_clock")
        self.rate = rospy.Rate(self.clock)
        self._main_start = False
        self._agent_util = AgentUtil()
        self.neighbor_id = rospy.get_param("neighbor_id")
        print(self.neighbor_id)
        self._my_zeta = np.zeros((1,3))
        self._zetas = np.zeros((3,3))

        self._pub_zeta = rospy.Publisher("zeta", PoseStamped, queue_size=1)
        rospy.Subscriber("/allZeta", PoseArray, self.zetaArrayCallback, queue_size=1)
        rospy.Subscriber("/joy", Joy, self.joy_callback, queue_size=1)
    

    def zetaArrayCallback(self, msg):
        # subscriber to get every agent's position
        arraynum = len(msg.poses)

        # elif self.checkNeighborStart:
        for i in range(arraynum):
            pos = [
                msg.poses[i].position.x,
                msg.poses[i].position.y,
                msg.poses[i].position.z,
            ]
            self._zetas[i] = pos

    def joy_callback(self, msg):
        ### START should be take off button and BACK should be land button
        if msg.buttons[XBoxButton.X]: # X button
            rospy.loginfo("main start")
            self._main_start = True

        self._joy_msg = msg

    ###################################################################
    ### start main

    def main_control(self):
        ##use drone's position to design the input ###
        myid = self._agent_util.get_my_id()
        my_position, my_orientation = self._agent_util.get_my_pose()
        all_positions = self._agent_util.get_all_positions()
        neighbor_positions = self._agent_util.get_neighbor_positions()

        #triangular    
        d1= np.array([(-1)*math.cos(math.pi/6),(-1)*math.sin(math.pi/6),0])  
        d2 = np.array([0,1,0])
        d3 = np.array([1*math.cos(math.pi/6),-1*math.sin(math.pi/6),0])
        q1= all_positions[0] + d1
        q2= all_positions[1] + d2
        q3= all_positions[2] + d3
        # print(q1, q2, q3)
        
        if myid==1:
            world = (q2-q1) + (q3-q1) 
        if myid==2:
            world = (q1-q2) + (q3-q2) 
        if myid==3:
            world = (q1-q3) + (q2-q3) 

        zeta_dot = world
        dt = 1.0 / self.clock
        self._my_zeta += zeta_dot * dt
        self.publish_zeta(self._my_zeta)
        rospy.loginfo(self._zetas)
        zeta1 = self._zetas[0]
        zeta2 = self._zetas[1]
        zeta3 = self._zetas[2]

        if myid==1:
            worldPI = 0.2* ((zeta1-zeta2)+(zeta1-zeta3))
        if myid==2:
            worldPI = 0.2 *((zeta2-zeta1)+(zeta2-zeta3))
        if myid==3:
            worldPI = 0.2 * ((zeta3-zeta1)+(zeta3-zeta2))

        #step3:joystick (all access by human)

        joy_ux = -self._joy_msg.axes[XBoxButton.LX]
        joy_uy = self._joy_msg.axes[XBoxButton.LY]
        omega_z = self._joy_msg.axes[XBoxButton.RX]  

        world_ux, world_uy, world_uz = world[0]+joy_ux+ worldPI[0], world[1]+joy_uy+worldPI[1], world[2] + worldPI[2]

        ## x y field limitation and collision avoidance with CBF
        #### x, y, z free 3 column
        neighbor_positions = self._agent_util.get_neighbor_positions()
        u_nom = np.array([world_ux, world_uy, world_uz,0,0,0])
        u_opt, w, status =self._cbf.solve(u_nom, my_position, neighbor_positions)
        
        world_ux = u_opt[0][0]
        world_uy = u_opt[1][0]
        world_uz = u_opt[2][0]

        quat = np.array(my_orientation)
        # print(quat)
        rotm_ = quaternion_matrix(quat)
        rotm = rotm_[0:3, 0:3]
        # print(rotm)
        omega_z = self._joy_msg.axes[XBoxButton.RX]
        body_vel = np.dot(rotm.transpose(), np.vstack([world_ux, world_uy, world_uz]))
        return body_vel[0], body_vel[1], body_vel[2], omega_z
    
    def publish_zeta(self, zeta):
        msg = PoseStamped()
        msg.pose.position.x = zeta[0][0]
        msg.pose.position.y = zeta[0][1]
        msg.pose.position.z = zeta[0][2]
        self._pub_zeta.publish(msg)

    def spin(self):
        # initialize message
        while not rospy.is_shutdown():
            ux = 0
            uy = 0
            omega_z = 0
            if self._main_start:
                ux, uy, omega_z = self.main_control()
            self._agent_util.publish_command(ux, uy, omega_z)
            self.rate.sleep()