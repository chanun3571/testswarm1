#!/usr/bin/env python

import actionlib, rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray,Pose,Quaternion,Twist,Point
from std_msgs.msg import Int32,String

class publish_goal_pose_to_robot2():
    def __init__(self):
        rospy.init_node('custom_waypoints2')
        rospy.loginfo('start robot2')
        rospy.Subscriber('/robot2_formation_pos', Pose, self.waypoint)
        rospy.Subscriber('/robot2/move_base/result',MoveBaseActionResult,self.failcallback2)
        self.flag = rospy.Publisher('/robot2/flag', String, queue_size=1)
        rospy.Subscriber('/swarm1/done', String, self.donecallback)
        self.done = "WAIT"
        self.locations = dict()
        self.flag2 = 0
        self.ready= False
    def donecallback(self,msg):
        self.done = msg.data
        # print(self.done)

        
    def waypoint(self, msg):
        self.pose = msg
        self.ready = True
    def sendGoals(self, waypoint):
        # subscribe to action server 
        client = actionlib.SimpleActionClient('robot2/move_base', MoveBaseAction)
        # this command to wait for the server to start listening for goals.
        client.wait_for_server()
        if self.ready:
            # Iterate over all the waypoits, follow the path 
            self.flag_done = "0"
            self.flag.publish(self.flag_done)
            
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = self.pose
            client.send_goal(goal)
            # rospy.loginfo(goal)
            wait = client.wait_for_result()
            if wait:
                self.flag_done = "1"
                self.flag.publish(self.flag_done)
                rospy.loginfo("robot2 done")
            while self.done != "DONE":
                self.done = self.done
                if self.done == "DONE":
                    break

    def failcallback2(self, msg):
        # if msg.status.text=="Failed to find a valid plan. Even after executing recovery behaviors.":
        rospy.loginfo(msg.status.text)
        if msg.status.text=="Robot is oscillating. Even after executing recovery behaviors." or \
           msg.status.text=="Failed to find a valid control. Even after executing recovery behaviors." or \
           msg.status.text=="Failed to find a valid plan. Even after executing recovery behaviors." :
            self.flag2 = 1
            rospy.loginfo("robot1")
            print(self.flag2)
            
    def resubmit2(self):
        if self.flag2 == 1:
            self.flag2 = 0
            self.sendGoals(self.locations)
            rospy.loginfo("resubmit robot #2")
            print(self.flag2)

    def spin(self):
        # initialize message
        while not rospy.is_shutdown():
            self.sendGoals(self.locations)
            self.resubmit2()
            rospy.Rate(5).sleep()

if __name__=='__main__':
    try:
        agent=publish_goal_pose_to_robot2()
        agent.spin()

    except rospy.ROSInterruptException:
        pass
