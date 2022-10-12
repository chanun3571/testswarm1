#!/usr/bin/env python3
import time
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class myrobot_goal():
    def __init__(self):
        rospy.init_node('move_my_robot')
        self.goal = [(1.5,-0.9,-1),(0.8,2.2,0.3),(-4,1.3,-1)]
        self.count = 0
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        wait = self.client.wait_for_server(rospy.Duration(5.0))
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return 
        rospy.loginfo("Connected to server")
        self.movebase_client()

    def movebase_client(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = self.goal[self.count][0]
        goal.target_pose.pose.position.y = self.goal[self.count][1]
        goal.target_pose.pose.orientation.w = self.goal[self.count][2]
        rospy.loginfo("Sending goal pose "+str(self.count+1)+" to Action Server")
        self.client.send_goal(goal,self.movebase_check) #,self.done_cb,self.active_cb, self.feedback_cb)
        rospy.spin()
    
    def movebase_check(self,y,w):
        self.count += 1
        rospy.loginfo("Goal pose "+str(self.count)+" reached") 
        #rospy.loginfo("Position"+"is"+[self.goal][self.count][0]+","+[self.goal][self.count][0]) 
        if self.count < len(self.goal):
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.pose.position.x = self.goal[self.count][0]
            goal.target_pose.pose.position.y = self.goal[self.count][1]
            goal.target_pose.pose.orientation.w = self.goal[self.count][2]
            rospy.loginfo("Sending goal pose "+str(self.count+1)+" to Action Server")
            self.client.send_goal(goal,self.movebase_check)
        else :
            rospy.loginfo("Final goal reached")
            self.movebase_back()

    def movebase_back(self):
        home = MoveBaseGoal()
        home.target_pose.header.frame_id = "map"
        home.target_pose.pose.position.x = 0
        home.target_pose.pose.position.y = 0
        home.target_pose.pose.orientation.w = 1
        self.client.send_goal(home)
        time.sleep(5.0)
        rospy.loginfo("Robot is going back to starting point")
        rospy.signal_shutdown("reason")
        
if __name__ == '__main__':
    try:
        myrobot_goal()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished")