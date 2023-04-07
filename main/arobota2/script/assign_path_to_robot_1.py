#!/usr/bin/env python

import actionlib, rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray,Pose,Quaternion,Twist,Point

class publish_goal_pose_to_robot1():
    def __init__(self):
        rospy.init_node('custom_waypoints1')
        rospy.Subscriber('/robot1_formation_pos', Point, self.CustomWayPoints1)
        self.locations = dict()

    def CustomWayPoints1(self, msg):
        # Create the dictionary 
        self.locations['waypoint1'] = Pose(msg, Quaternion(0.000, 0.000, -0.717, 0.697))

    def sendGoals(self, waypoints):
        # subscribe to action server 
        client = actionlib.SimpleActionClient('robot1/move_base', MoveBaseAction)
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
        print(goal)

    def spin(self):
        # initialize message
        self.sendGoals(self.locations)
        while not rospy.is_shutdown():
            rate = rospy.Rate(10)
            rate.sleep()

if __name__=='__main__':
    try:
        agent=publish_goal_pose_to_robot1()
        agent.spin()

    except rospy.ROSInterruptException:
        pass
