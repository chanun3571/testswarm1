#!/usr/bin/env python

import actionlib, rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal, MoveBaseActionResult, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray,Pose,Quaternion,Twist,Point, PoseStamped
from std_msgs.msg import String, Int32
from actionlib_msgs.msg import GoalID

class cancel_goal():
    def __init__(self):
        rospy.init_node('wait_to_cancel_node')
        self._uh = Twist()
        # rospy.Subscriber('move_base/goal', MoveBaseActionGoal, self.CustomWayPoints1, queue_size=1)
        # rospy.Subscriber('move_base/result',MoveBaseActionResult,self.failcallback1, queue_size=1)
        rospy.Subscriber('/depth', String, self.depth_callback, queue_size=1)
        rospy.Subscriber('/x',String, self.x_callback, queue_size=1)
        rospy.Subscriber('/camera_status', String, self.camera_status, queue_size=1)
        self.pubinterrupt = rospy.Publisher('interruptsignal', String, queue_size=1)
        self.cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
        self.cancel_msg = GoalID()
        self.camstat = "WAITING"
        rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.allpose_callback)
        self.pubvel =  rospy.Publisher('cmd_vel',Twist, queue_size=10)
        # self.pubvel1 = rospy.Publisher('robot1/cmd_vel',Twist, queue_size=10)
        # self.pubvel2 = rospy.Publisher('robot2/cmd_vel',Twist, queue_size=10)
        # self.pubvel3 = rospy.Publisher('robot3/cmd_vel',Twist, queue_size=10)
    def moveforward(self):
        self.joy_ux = 0.3
        self.joy_uy = 0
        self.joy_omega = 0
        self._uh.linear.x = self.joy_ux #(-1,1)
        self._uh.linear.y = self.joy_uy #(-1,1)
        self._uh.angular.z = self.joy_omega #(-1,1)
        self.pubvel.publish(self._uh)
        # self.pubvel1.publish(self._uh)
        # self.pubvel2.publish(self._uh)
        # self.pubvel3.publish(self._uh)

    def stopmotion(self):
        self.joy_ux = 0
        self.joy_uy = 0
        self.joy_omega = 0
        self._uh.linear.x = self.joy_ux #(-1,1)
        self._uh.linear.y = self.joy_uy #(-1,1)
        self._uh.angular.z = self.joy_omega #(-1,1)
        self.pubvel.publish(self._uh)
        # self.pubvel1.publish(self._uh)
        # self.pubvel2.publish(self._uh)
        # self.pubvel3.publish(self._uh)
    def counterrotate(self):
        self.joy_ux = 0
        self.joy_uy = 0
        self.joy_omega = 3
        self._uh.linear.x = self.joy_ux #(-1,1)
        self._uh.linear.y = self.joy_uy #(-1,1)
        self._uh.angular.z = self.joy_omega #(-1,1)
        self.pubvel.publish(self._uh)
        # self.pubvel1.publish(self._uh)
        # self.pubvel2.publish(self._uh)
        # self.pubvel3.publish(self._uh)
    def rotate(self):
        self.joy_ux = 0
        self.joy_uy = 0
        self.joy_omega = -3
        self._uh.linear.x = self.joy_ux #(-1,1)
        self._uh.linear.y = self.joy_uy #(-1,1)
        self._uh.angular.z = self.joy_omega #(-1,1)
        self.pubvel.publish(self._uh)
        # self.pubvel1.publish(self._uh)
        # self.pubvel2.publish(self._uh)
        # self.pubvel3.publish(self._uh)

    def allpose_callback(self, msg):       
        self.robot = msg.pose.pose.position

    def x_callback(self,msg):
        self.x = msg.data

    def depth_callback(self,msg):
        self.depth = msg.data 

    def camera_status(self,msg):
        self.camstat = msg.data

    def navtoball(self):
            # rate = rospy.Rate()
            print(self.camstat)
            self.stopmotion()
            # print(self.depth)
            while self.camstat == "tracking":
                # rate.sleep()
                # if self.camstat == "not_found":
                #     self.stopmotion()
                #     break
                # if self.camstat == "not_tracking":
                #     if self.x < 10:
                #         self.rotate()
                #     if self.x > 18:
                #         self.counterrotate()
                if float(self.depth)<45:
                    self.stopmotion()
                    break
                if float(self.depth)>45:
                    self.stopmotion()
                    break
                if self.camstat != "tracking":
                    self.stopmotion()

    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.camstat == "tracking":
                self.pubinterrupt.publish("STOP")
                print("canceled goal")
                # self.cancel_pub.publish(self.cancel_msg)
                self.navtoball()
                rate.sleep()
                break
                
if __name__=='__main__':
    try:
        agent=cancel_goal()
        agent.spin()
    except rospy.ROSInterruptException:
        pass
