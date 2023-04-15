#!/usr/bin/env python

import actionlib, rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal, MoveBaseActionResult, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray,Pose,Quaternion,Twist,Point

class publish_goal_pose_to_robot1():
    def __init__(self):
        rospy.init_node('custom_waypoints1')
        rospy.Subscriber('move_base/goal', MoveBaseActionGoal, self.CustomWayPoints1, queue_size=1)
        rospy.Subscriber('move_base/result',MoveBaseActionResult,self.failcallback1, queue_size=1)
        self.locations = dict()
        self.flag1 = 0

    def CustomWayPoints1(self, msg):
        # Create the dictionary 
        self.locations['waypoint1'] = Pose(msg.goal.target_pose.pose.position, msg.goal.target_pose.pose.orientation)

    def sendGoals(self, waypoints):
        # subscribe to action server 
        client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        # this command to wait for the server to start listening for goals.
        client.wait_for_server()
        
        # Iterate over all the waypoits, follow the path 
        for key, value in waypoints.items():
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()

            goal.target_pose.pose.position.x = waypoints[key].position.x
            goal.target_pose.pose.position.y = waypoints[key].position.y
            goal.target_pose.pose.position.z = waypoints[key].position.z
            # Goal Orientation
            goal.target_pose.pose.orientation.x = waypoints[key].orientation.x
            goal.target_pose.pose.orientation.y = waypoints[key].orientation.y
            goal.target_pose.pose.orientation.w = waypoints[key].orientation.z
            goal.target_pose.pose.orientation.z = waypoints[key].orientation.w

            client.send_goal(goal)
            wait = client.wait_for_result()
        # print(goal)

    def failcallback1(self, msg):
        # if msg.status.text=="Failed to find a valid plan. Even after executing recovery behaviors.":
        rospy.loginfo(msg.status.text)
        if msg.status.text=="Robot is oscillating. Even after executing recovery behaviors." or \
           msg.status.text=="Failed to find a valid control. Even after executing recovery behaviors." or \
           msg.status.text=="Failed to find a valid plan. Even after executing recovery behaviors." :
            self.flag1 = 1
            # rospy.loginfo("robot1")
            print(self.flag1)
        # if msg.status.text== "Goal reached." :
        #     self.flag1 = 0
            
    def resubmit1(self):
        if self.flag1 == 1:
            self.flag1 = 0
            self.sendGoals(self.locations)
            rospy.loginfo(self.locations)
            rospy.loginfo("resubmit robot #1")
            print(self.flag1)
            # rospy.Rate(0.5).sleep()
            # self.flag1 = 0
            print(self.flag1)

    def spin(self):
        # initialize message
        rate = rospy.Rate(20)
        # self.sendGoals(self.locations)
        while not rospy.is_shutdown():
            self.resubmit1()
            rate.sleep()

if __name__=='__main__':
    try:
        agent=publish_goal_pose_to_robot1()
        agent.spin()

    except rospy.ROSInterruptException:
        pass
