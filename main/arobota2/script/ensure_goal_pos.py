#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose,Twist,PoseStamped,Vector3
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult


class ResubmitGoalPose():
    def __init__(self):
        self.flag1 = 0
        self.flag2 = 0
        self.flag3 = 0
        rospy.init_node('joystickinput1',anonymous=True)
        rospy.Subscriber('/robot1/move_base/result',MoveBaseActionResult,self.failcallback1)
        rospy.Subscriber('/robot2/move_base/result',MoveBaseActionResult,self.failcallback2)
        rospy.Subscriber('/robot3/move_base/result',MoveBaseActionResult,self.failcallback3)
        rospy.Subscriber('/robot1/move_base_simple/goal',PoseStamped,self.resubmitcallback1)
        rospy.Subscriber('/robot2/move_base_simple/goal',PoseStamped,self.resubmitcallback2)
        rospy.Subscriber('/robot3/move_base_simple/goal',PoseStamped,self.resubmitcallback3)
        self.pub1 = rospy.Publisher('/robot1/move_base_simple/goal',PoseStamped, queue_size=10)
        self.pub2 = rospy.Publisher('/robot2/move_base_simple/goal',PoseStamped, queue_size=10)
        self.pub3 = rospy.Publisher('/robot3/move_base_simple/goal',PoseStamped, queue_size=10)


    def failcallback1(self, msg):
        if msg.status.text=="Failed to find a valid plan. Even after executing recovery behaviors.":
            self.flag1 = 1
            self.resubmit1()
    def failcallback2(self, msg):
        if msg.status.text=="Failed to find a valid plan. Even after executing recovery behaviors.":
            self.flag2 = 1
            self.resubmit2()
    def failcallback3(self, msg):
        if msg.status.text=="Failed to find a valid plan. Even after executing recovery behaviors.":
            self.flag3 = 1
            self.resubmit3()
    def resubmitcallback1(self, msg):
            self.goal1 = msg
    def resubmitcallback2(self, msg):
            self.goal2 = msg
    def resubmitcallback3(self, msg):
            self.goal3 = msg
    def resubmit1(self):
        if self.flag1 == 1:
            self.pub1.publish(self.goal1)
            rospy.loginfo("resubmit robot #1")
            self.flag1 = 0
    def resubmit2(self):
        if self.flag2 == 1:
            self.pub2.publish(self.goal2)
            rospy.loginfo("resubmit robot #2")
            self.flag2 = 0
    def resubmit3(self):
        if self.flag3 == 1:
            self.pub3.publish(self.goal3)
            rospy.loginfo("resubmit robot #3")
            self.flag3 = 0                 

if __name__=='__main__':
    try:
        ResubmitGoalPose()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
