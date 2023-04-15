#!/usr/bin/env python

import actionlib, rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray,Pose,Quaternion,Twist,Point

class publish_goal_pose_to_robot3():
    def __init__(self):
        rospy.init_node('custom_waypoints3')
        rospy.Subscriber('/robot3_formation_pos', Point, self.CustomWayPoints3)
        rospy.Subscriber('/robot3/move_base/result',MoveBaseActionResult,self.failcallback3)
        self.locations = dict()
        self.flag3 = 0

    def CustomWayPoints3(self, msg):
        # Create the dictionary 
        self.locations['waypoint1'] = Pose(msg, Quaternion(0.000, 0.000, -0.717, 0.697))

    def sendGoals(self, waypoints):
        # subscribe to action server 
        client = actionlib.SimpleActionClient('robot3/move_base', MoveBaseAction)
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

    def failcallback3(self, msg):
        rospy.loginfo(msg.status.text)
        # if msg.status.text=="Failed to find a valid plan. Even after executing recovery behaviors.":
        if msg.status.text=="Robot is oscillating. Even after executing recovery behaviors." or \
           msg.status.text=="Failed to find a valid control. Even after executing recovery behaviors." or \
           msg.status.text=="Failed to find a valid plan. Even after executing recovery behaviors." :
            self.flag3 = 1
            rospy.loginfo("robot3")
    def resubmit3(self):
        if self.flag3 == 1:
            self.flag3 = 0
            self.sendGoals(self.locations)
            rospy.loginfo("resubmit robot #3")

    def spin(self):
        # initialize message
        self.sendGoals(self.locations)
        while not rospy.is_shutdown():
            self.resubmit3()

if __name__=='__main__':
    try:
        agent=publish_goal_pose_to_robot3()
        agent.spin()

    except rospy.ROSInterruptException:
        pass
