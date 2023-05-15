#!/usr/bin/env python

import actionlib, rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray,Pose,Quaternion,Twist,Point
from std_msgs.msg import Int32,String

class publish_goal_pose_to_robot1():
    def __init__(self):
        rospy.init_node('custom_waypoints1')
        rospy.loginfo('start robot1')
        rospy.Subscriber('/robot1_formation_pos', Pose, self.CustomWayPoints1)
        rospy.Subscriber('/robot1/move_base/result',MoveBaseActionResult,self.failcallback1)
        self.flag = rospy.Publisher('/robot1/flag', String, queue_size=1)
        rospy.Subscriber('/swarm1/done', String, self.donecallback)
        self.done = "WAIT"
        self.locations = dict()
        self.flag1 = 0
        
    def donecallback(self,msg):
        self.done = msg

        
    def CustomWayPoints1(self, msg):
        # Create the dictionary 
        self.locations['robot1'] = msg
        # rospy.loginfo(self.locations)

    def sendGoals(self, waypoints):
        # subscribe to action server 
        client = actionlib.SimpleActionClient('robot1/move_base', MoveBaseAction)
        # this command to wait for the server to start listening for goals.
        client.wait_for_server()
        
        # Iterate over all the waypoits, follow the path 
        for key, value in waypoints.items():
            self.flag_done = "0"
            self.flag.publish(self.flag_done)
            
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()

            goal.target_pose.pose.position.x = waypoints[key].position.x
            goal.target_pose.pose.position.y = waypoints[key].position.y
            goal.target_pose.pose.position.z = waypoints[key].position.z
            # Goal Orientation
            goal.target_pose.pose.orientation.x = waypoints[key].orientation.x
            goal.target_pose.pose.orientation.y = waypoints[key].orientation.y
            goal.target_pose.pose.orientation.z = waypoints[key].orientation.z
            goal.target_pose.pose.orientation.w = waypoints[key].orientation.w

            client.send_goal(goal)
            rospy.loginfo(goal)
            wait = client.wait_for_result()
            self.flag_done = "1"
            self.flag.publish(self.flag_done)
            rospy.loginfo("robot1 done")
            while self.done != "DONE":
                self.x=1
                # print("waiting..")
            else:
                continue

            
        # self.locations = dict()
        

    def failcallback1(self, msg):
        # if msg.status.text=="Failed to find a valid plan. Even after executing recovery behaviors.":
        rospy.loginfo(msg.status.text)
        if msg.status.text=="Robot is oscillating. Even after executing recovery behaviors." or \
           msg.status.text=="Failed to find a valid control. Even after executing recovery behaviors." or \
           msg.status.text=="Failed to find a valid plan. Even after executing recovery behaviors." :
            self.flag1 = 1
            rospy.loginfo("robot1")
            print(self.flag1)
            
    def resubmit1(self):
        if self.flag1 == 1:
            self.flag1 = 0
            self.sendGoals(self.locations)
            rospy.loginfo("resubmit robot #1")
            print(self.flag1)

    def spin(self):
        # initialize message
        while not rospy.is_shutdown():
            self.sendGoals(self.locations)
            self.resubmit1()
            rospy.Rate(20).sleep()

if __name__=='__main__':
    try:
        agent=publish_goal_pose_to_robot1()
        agent.spin()

    except rospy.ROSInterruptException:
        pass
