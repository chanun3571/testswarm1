#!/usr/bin/env python

import actionlib, rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal, MoveBaseActionResult, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray,Pose,Quaternion,Twist,Point, PoseStamped
from std_msgs.msg import String, Int32
import time 


class publish_goal_pose_to_robot():
    def __init__(self):
        rospy.init_node('custom_waypoints1')
        rospy.Subscriber('robot1/flag',String,self.robot1flagcallback, queue_size=1)
        rospy.Subscriber('robot2/flag',String,self.robot2flagcallback, queue_size=1)
        rospy.Subscriber('robot3/flag',String,self.robot3flagcallback, queue_size=1)
        self.pubgoal = rospy.Publisher('/swarm1/move_base_simple/goal', PoseStamped, queue_size=1)
        self.pubsend = rospy.Publisher('/swarm1/done', String, queue_size=10)
        # self.totalflag = 0 
        # self.flag1 = 0
        # self.flag2 = 0
        # self.flag3 = 0
        # self.flag = False

    def CustomWayPoints(self):
        # Create the dictionary 
        self.locations = dict()
        self.locations['waypoint1'] = Point(-1, 0.7, 0.000)
        self.locations['waypoint2'] = Point(-1, 0.6, 0.000)
        self.locations['waypoint3'] = Point(-1, 0.5, 0.000)
        self.locations['waypoint4'] = Point(-1, 0.4, 0.000)
        self.locations['waypoint5'] = Point(-1, 0.3, 0.000)
        self.locations['waypoint6'] = Point(-1, 0.2, 0.000)
        self.locations['waypoint7'] = Point(-1, 0.1, 0.000)
        self.locations['waypoint8'] = Point(-1, 0.0, 0.000)
        self.locations['waypoint9'] = Point(-0.9, 0.0, 0.000)        
        self.locations['waypoint10'] = Point(-0.8, 0.0, 0.000)
        self.locations['waypoint11'] = Point(-0.7, 0.0, 0.000)
        self.locations['waypoint12'] = Point(-0.6, 0.0, 0.000)
        self.locations['waypoint9'] = Point(-0.5, -0.2, 0.000)        
        self.locations['waypoint10'] = Point(-0.5, -0.1, 0.000)
        self.locations['waypoint11'] = Point(-0.5, 0.0, 0.000)
        self.locations['waypoint12'] = Point(-0.5, 0.1, 0.000)

    def sendGoals(self, waypoints):
        for key, value in waypoints.items():
            self.flag1 = 0
            self.flag2 = 0
            self.flag3 = 0
            self.flag = False
            self.goal = PoseStamped()
            self.goal.header.frame_id = "map"
            self.goal.pose.position.x = waypoints[key].x
            self.goal.pose.position.y = waypoints[key].y
            self.goal.pose.position.z = waypoints[key].z
            self.pubgoal.publish(self.goal)
            # print(self.goal)
            while not self.flag:
                print(self.totalflag)
                rospy.Rate(5).sleep()
                self.pubgoal.publish(self.goal)
                self.checktotalflag(self.flag1, self.flag2,self.flag3) 
                # print(self.flag1,self.flag2,self.flag3)
                if self.flag:
                    break

    def robot1flagcallback(self,msg):
        self.flag1 = int(msg.data)
    def robot2flagcallback(self,msg):
        self.flag2 = int(msg.data)
    def robot3flagcallback(self,msg):
        self.flag3 = int(msg.data)

    def checktotalflag(self,flag1,flag2,flag3):
        self.totalflag = flag1+flag2+flag3
        if self.totalflag != 3:
            self.flag = False
        if self.totalflag == 3:
            self.pubsend.publish("DONE")
            self.flag= True
            print("next goal")

    def spin(self):
        # initialize message
        self.CustomWayPoints()
        self.sendGoals(self.locations)
        while not rospy.is_shutdown():
            pass

if __name__=='__main__':
    try:
        agent=publish_goal_pose_to_robot()
        agent.spin()
    except rospy.ROSInterruptException:
        pass
