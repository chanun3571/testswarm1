#!/usr/bin/env python

import actionlib, rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray,Pose,Quaternion,Twist,Point

rospy.init_node('custom_waypoints2')

def CustomWayPoints2():
    # Create the dictionary 
    locations = dict()
    # add our waypoint names and values. 
    locations['waypoint1'] = Pose(Point(0, 0, 0.000), Quaternion(0.000, 0.000, -0.717, 0.697))
    locations['waypoint2'] = Pose(Point(1.1, -0.827, 0.000),Quaternion(0.000, 0.000, -0.707, 0.708))
    locations['waypoint3'] = Pose(Point(-1.2, -0.3, 0.000), Quaternion(0.000, 0.000, -0.016, 1.000))
    return locations

# create a function that represnt posearray in Rviz so we could visualize the waypoints 
def wayPointsRviz(waypointsList):
    poseArrayPub= rospy.Publisher('/waypoints', PoseArray, queue_size=1)
    waypoints = PoseArray()
    waypoints.header.frame_id = 'map'
    waypointPoses = []
    for key, value in waypointsList.items():
        waypointPoses.append(waypointsList[key])

    waypoints.poses = waypointPoses
    poseArrayPub.publish(waypoints)
    return waypoints


def sendGoals(waypoints):
    # subscribe to action server 
    client = actionlib.SimpleActionClient('robot2/move_base', MoveBaseAction)
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
    rospy.loginfo('The waypoints path is complete')

ss = CustomWayPoints2()

send = sendGoals(ss)
print(wayPointsRviz(ss))