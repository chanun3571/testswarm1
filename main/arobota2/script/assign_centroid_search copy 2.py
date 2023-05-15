#!/usr/bin/env python

import actionlib, rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal, MoveBaseActionResult, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray,Pose,Quaternion,Twist,Point, PoseStamped
from std_msgs.msg import String, Int32

class publish_goal_pose_to_robot():
    def __init__(self):
        rospy.init_node('custom_waypoints1')
        rospy.Subscriber('robot1/flag',String,self.robot1flagcallback, queue_size=10)
        rospy.Subscriber('robot2/flag',String,self.robot2flagcallback, queue_size=10)
        rospy.Subscriber('robot3/flag',String,self.robot3flagcallback, queue_size=10)
        self.pubgoal = rospy.Publisher('/swarm1/move_base_simple/goal', PoseStamped, queue_size=10)
        self.pubsend = rospy.Publisher('/swarm1/done', String, queue_size=10)
        self.locations = dict()
        self.totalflag = 0 
        self.flag1 = 0
        self.flag2 = 0
        self.flag3 = 0
        self.flag = False
    def CustomWayPoints(self):
        # Create the dictionary 
        self.locations['waypoint1'] = Pose(Point(-1, 0.7, 0.000), Quaternion(0.000, 0.000, -0.717, 0.697))
        self.locations['waypoint2'] = Pose(Point(-1, -0.2, 0.000),Quaternion(0.000, 0.000, -0.717, 0.697))
        self.locations['waypoint3'] = Pose(Point(-0.8, -0.2, 0.000), Quaternion(0.000, 0.000, -0.717, 0.697))
        # self.locations['waypoint4'] = Pose(Point(-0.8, 0.2, 0.000), Quaternion(0.000, 0.000, -0.717, 0.697))

    def sendGoals(self, waypoints):
        for key, value in waypoints.items():
            self.flag = False
            self.goal = PoseStamped()
            self.goal.header.frame_id = "map"
            self.goal.pose.position.x = waypoints[key].position.x
            self.goal.pose.position.y = waypoints[key].position.y
            self.goal.pose.position.z = waypoints[key].position.z
            self.pubgoal.publish(self.goal)
            print("sent")
            print(self.goal)
            while not self.flag:
                self.pubgoal.publish(self.goal)
                self.checktotalflag(self.flag1, self.flag2,self.flag3) 
                print(self.flag1,self.flag2,self.flag3)
                if self.flag:
                    self.checktotalflag(self.flag1, self.flag2,self.flag3) 
                    break            

    def robot1flagcallback(self,msg):
        self.flag1 = float(msg.data)
    def robot2flagcallback(self,msg):
        self.flag2 = float(msg.data)
    def robot3flagcallback(self,msg):
        self.flag3 = float(msg.data)

    def checktotalflag(self,flag1,flag2,flag3):
        self.totalflag = flag1+flag2+flag3
        if self.totalflag != 3:
            self.flag=False
            print(self.totalflag)
            # print("waiting...")
        else:
            self.flag=True
            self.pubsend.publish("DONE")
            print("next goal")
            self.pubsend.publish("Waiting")
            print("waiting")

    def spin(self):
        # initialize message
        self.CustomWayPoints()
        print(self.locations)
        self.sendGoals(self.locations)
        print(self.goal)
        rate = rospy.Rate(20)
        # self.sendGoals(self.locations)
        while not rospy.is_shutdown():
            self.sendGoals(self.locations)
            rate.sleep()

if __name__=='__main__':
    try:
        agent=publish_goal_pose_to_robot()
        agent.spin()
    except rospy.ROSInterruptException:
        pass
