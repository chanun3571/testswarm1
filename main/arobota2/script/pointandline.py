#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray,Pose,Quaternion,Twist,Point
from std_msgs.msg import String
class createvisuallinepoint:
    def __init__(self):
        rospy.init_node('create_visual_line',anonymous=True)
        self.marker_pub = rospy.Publisher('visual_line', Marker, queue_size=1) 
        rospy.Subscriber("/robot1/amcl_pose", PoseWithCovarianceStamped, self.allpose1_callback)
        rospy.Subscriber("/robot2/amcl_pose", PoseWithCovarianceStamped, self.allpose2_callback)
        rospy.Subscriber("/robot3/amcl_pose", PoseWithCovarianceStamped, self.allpose3_callback)
        rospy.Subscriber('initialize_state', String, self.robotinitdone, queue_size=10)

        self.robot1 = Point()
        self.robot2 = Point()
        self.robot3 = Point()
        self.initdone = "WAIT"

    def robotinitdone(self, msg):
        self.initdone = msg.data

        print(msg.data)
    def allpose1_callback(self, msg):       
        self.robot1 = msg.pose.pose.position
    def allpose2_callback(self, msg):
        self.robot2 = msg.pose.pose.position
    def allpose3_callback(self, msg):
        self.robot3 = msg.pose.pose.position

    def point_callback(self, msg):
        self.point = msg

    def formation_marker(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD

        # marker scale
        marker.scale.x = 0.02
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        # marker color
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        # marker orientaiton
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = -1.0

        # marker position
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.02

        # marker line points
        marker.points = []
        # first point
        first_line_point = self.robot1
        second_line_point = self.robot2
        third_line_point = self.robot3
        forth_line_point = self.robot1
        marker.points.append(first_line_point)
        marker.points.append(second_line_point)
        marker.points.append(third_line_point)
        marker.points.append(forth_line_point)

        # Publish the Marker
        self.marker_pub.publish(marker)

    def spin(self):
        # initialize message
        while not rospy.is_shutdown():
            if self.initdone == "DONE":
                self.formation_marker()

if __name__=='__main__':
    try:
        agent=createvisuallinepoint()
        agent.spin()
    except rospy.ROSInterruptException:
        pass

