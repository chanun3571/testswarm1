#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose,Twist,PoseStamped,Vector3
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult, MoveBaseActionFeedback
from actionlib_msgs.msg import GoalStatusArray

class ResubmitGoalPose():
    def __init__(self):
        rospy.init_node('ensure_goal', anonymous=True)
        self.flag1 = 0
        self.flag2 = 0
        self.flag3 = 0
        # rospy.init_node('joystickinput1',anonymous=True)
        rospy.Subscriber('/robot1/move_base/result',MoveBaseActionResult,self.failcallback1)
        rospy.Subscriber('/robot2/move_base/result',MoveBaseActionResult,self.failcallback2)
        rospy.Subscriber('/robot3/move_base/result',MoveBaseActionResult,self.failcallback3)
        # rospy.Subscriber('/robot1/move_base/feedback',MoveBaseActionFeedback,self.failcallback1)
        # rospy.Subscriber('/robot2/move_base/feedback',MoveBaseActionFeedback,self.failcallback2)
        # rospy.Subscriber('/robot3/move_base/feedback',MoveBaseActionFeedback,self.failcallback3)
        rospy.Subscriber('/robot1/move_base_simple/goal',PoseStamped,self.resubmitcallback1)
        rospy.Subscriber('/robot2/move_base_simple/goal',PoseStamped,self.resubmitcallback2)
        rospy.Subscriber('/robot3/move_base_simple/goal',PoseStamped,self.resubmitcallback3)
        self.pub1 = rospy.Publisher('/robot1/move_base_simple/goal',PoseStamped, queue_size=10)
        self.pub2 = rospy.Publisher('/robot2/move_base_simple/goal',PoseStamped, queue_size=10)
        self.pub3 = rospy.Publisher('/robot3/move_base_simple/goal',PoseStamped, queue_size=10)
        self.goal1 = PoseStamped()
        self.goal2 = PoseStamped()
        self.goal3 = PoseStamped()

    def failcallback1(self, msg):
        # if msg.status.text=="Failed to find a valid plan. Even after executing recovery behaviors.":
        if msg.status.text=="Robot is oscillating. Even after executing recovery behaviors." or msg.status.text=="Failed to find a valid plan. Even after executing recovery behaviors.":
            self.flag1 = 1
            rospy.loginfo("robot1")
    def failcallback2(self, msg):
        # if msg.status.text=="Failed to find a valid plan. Even after executing recovery behaviors.":
        if msg.status.text=="Robot is oscillating. Even after executing recovery behaviors." or msg.status.text=="Failed to find a valid plan. Even after executing recovery behaviors.":
            self.flag2  = 1
            rospy.loginfo("robot2")
    def failcallback3(self, msg):
        # if msg.status.text=="Failed to find a valid plan. Even after executing recovery behaviors.":
        if msg.status.text=="Robot is oscillating. Even after executing recovery behaviors." or msg.status.text=="Failed to find a valid plan. Even after executing recovery behaviors.":
            self.flag3 = 1
            rospy.loginfo("robot3")
    def resubmitcallback1(self, msg):
        rospy.loginfo(msg)
        self.goal1 = msg
    def resubmitcallback2(self, msg):
        rospy.loginfo(msg)
        self.goal2 = msg
    def resubmitcallback3(self, msg):
        rospy.loginfo(msg)
        self.goal3 = msg

    def resubmit1(self):
        self.pub1.publish(self.goal1)
        rospy.loginfo("resubmit robot #1")
        self.flag1 = 0
    def resubmit2(self):
        self.pub2.publish(self.goal2)
        rospy.loginfo("resubmit robot #2")
        self.flag2 = 0
    def resubmit3(self):
        self.pub3.publish(self.goal3)
        rospy.loginfo("resubmit robot #3")
        self.flag3 = 0                 

    def spin(self):
        # initialize message
        while not rospy.is_shutdown():
            rospy.Rate(10).sleep()
            if self.flag1 == 1:
                self.resubmit1()
            if self.flag2 == 1:
                self.resubmit2()
            if self.flag3 == 1:
                self.resubmit3()



if __name__=='__main__':
    try:
        ResubmitGoalPose().spin()
    except rospy.ROSInterruptException:
        pass
