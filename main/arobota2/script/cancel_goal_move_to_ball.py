#!/usr/bin/env python

import actionlib, rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal, MoveBaseActionResult, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray,Pose,Quaternion,Twist,Point, PoseStamped
from std_msgs.msg import String, Int32
from actionlib_msgs.msg import GoalID

class publish_goal_pose_to_robot1():
    def __init__(self):
        rospy.init_node('custom_waypoints1')
        rospy.Subscriber('move_base/goal', MoveBaseActionGoal, self.CustomWayPoints1, queue_size=1)
        rospy.Subscriber('move_base/result',MoveBaseActionResult,self.failcallback1, queue_size=1)
        rospy.Subscriber('depth', String, self.depthcallback(), queue_size=1)
        rospy.Subscriber('x',String,self.failcallback1, queue_size=1)
        rospy.Pub
        rospy.Subscriber('/camera_status', String, self.camera_status, queue_size=10)
        self.cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
        self.cancel_msg = GoalID()
        self.camstat = ""
        rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.allpose_callback)
        self.pubvel =  rospy.Publisher('cmd_vel',Twist, queue_size=10)
        # self.pubvel1 = rospy.Publisher('robot1/cmd_vel',Twist, queue_size=10)
        # self.pubvel2 = rospy.Publisher('robot2/cmd_vel',Twist, queue_size=10)
        # self.pubvel3 = rospy.Publisher('robot3/cmd_vel',Twist, queue_size=10)

    def moveforward(self):
        self.joy_ux = 3
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
        joy_ux = 0
        joy_uy = 0
        joy_omega = 0
        self._uh.linear.x = joy_ux #(-1,1)
        self._uh.linear.y = joy_uy #(-1,1)
        self._uh.angular.z = joy_omega #(-1,1)
        self.pubvel.publish(self._uh)
        # self.pubvel1.publish(self._uh)
        # self.pubvel2.publish(self._uh)
        # self.pubvel3.publish(self._uh)

    def allpose_callback(self, msg):       
        self.robot = msg.pose.pose.position

    def camera_status(self,msg):
        self.camstat = msg.data

    def navtoball(self):
        self.stopmotion()
        self.moveforward()
        if depth
        
    def spin(self):s
        while not rospy.is_shutdown():
            if self.camstat == "tracking":
                self.cancel_pub.publish(self.cancel_msg)
                self.navtoball(self)
                

if __name__=='__main__':
    try:
        agent=publish_goal_pose_to_robot1()
        agent.spin()
    except rospy.ROSInterruptException:
        pass
