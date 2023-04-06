#!/usr/bin/env python

import actionlib, rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray,Pose,Quaternion,Twist,Point

rospy.init_node('allPose')
class PoseCollector:
    def __init__(self):
        rospy.Subscriber("/robot1/amcl_pose", PoseWithCovarianceStamped, self.allpose1_callback)
        rospy.Subscriber("/robot2/amcl_pose", PoseWithCovarianceStamped, self.allpose2_callback)
        rospy.Subscriber("/robot3/amcl_pose", PoseWithCovarianceStamped, self.allpose3_callback)
        self.poseArrayPub = rospy.Publisher('/allpose', PoseArray, queue_size=1)
        self.locations = dict()

    def allpose1_callback(self, msg):
        self.locations['robot1'] = msg.pose.pose
    def allpose2_callback(self, msg):
        self.locations['robot2'] = msg.pose.pose
    def allpose3_callback(self, msg):
        self.locations['robot3'] = msg.pose.pose

# create a function that represnt posearray in Rviz so we could visualize the waypoints 
    def wayPoints(self, waypointsList):
        self.waypoints = PoseArray()
        self.waypoints.header.frame_id = 'map'
        self.waypoints.header.stamp= rospy.Time.now()
        self.waypointPoses = []
        for key, value in waypointsList.items():
            self.waypointPoses.append(waypointsList[key])
        self.waypoints.poses = self.waypointPoses
        self.poseArrayPub.publish(self.waypoints)

    def spin(self):
        # initialize message
        while not rospy.is_shutdown():
            rate = rospy.Rate(10)
            self.wayPoints(self.locations)
            rate.sleep()
            
if __name__=='__main__':
    try:
        agent=PoseCollector()
        agent.spin()
    except rospy.ROSInterruptException:
        pass
