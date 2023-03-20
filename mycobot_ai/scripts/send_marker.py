#!/usr/bin/env python3

# encoding: utf-8

import rospy
import time
from visualization_msgs.msg import Marker

class Send_marker(object):
    def __init__(self):
        # Inherit object class objects
        super(Send_marker, self).__init__()
        # Initialize a node. If the node is not created, the information cannot be published
        rospy.init_node("send_marker", anonymous=True)
        # Create a publisher to publish marker
        self.pub = rospy.Publisher("/cube", Marker, queue_size=1)
        # Create a block model that marker uses to create
        self.marker = Marker()
        # Configure its ownership relationship, and its coordinates are relative to / joint1.
        # /Joint1 represents the bottom of the manipulator in the model.
        self.marker.header.frame_id = "base"
        # Set marker's name
        self.marker.ns = "cube"
        # Set the type of marker to be square
        self.marker.type = self.marker.CUBE
        # Set marker's action to add (marker without this name will add one)
        self.marker.action = self.marker.ADD
        # Set the actual size of marker, in m
        self.marker.scale.x = 0.04
        self.marker.scale.y = 0.04
        self.marker.scale.z = 0.04
        # Set marker's color, 1.0 for 255 (this represents a ratio conversion)
        self.marker.color.a = 1.0
        self.marker.color.g = 1.0
        self.marker.color.r = 1.0
        # Initialize marker's position and its four-dimensional attitude
        self.marker.pose.position.x = 0.1
        self.marker.pose.position.y = 0
        self.marker.pose.position.z = 0.03
        self.marker.pose.orientation.x = 0
        self.marker.pose.orientation.y = 0
        self.marker.pose.orientation.z = 0
        self.marker.pose.orientation.w = 1.0

    # Modify coordinates and publish marker
    def pub_marker(self, x, y, z=0.03):
        # Set marker's timestamp
        self.marker.header.stamp = rospy.Time.now()
        # Set marker's spatial coordinates
        self.marker.pose.position.x = x
        self.marker.pose.position.y = y
        self.marker.pose.position.z = z
        # Release marker
        self.pub.publish(self.marker)

    # Let marker happen y
    def run(self):
        time.sleep(1) 
        self.pub_marker(0.1, -0.1)
        time.sleep(1)
        self.pub_marker(0.15, 0)
        time.sleep(1)
        self.pub_marker(0.15, -0.1)
        time.sleep(1)
        self.pub_marker(0.2, -0.1)
        time.sleep(1)
        self.pub_marker(0.15, -0.05)
        time.sleep(1)


if __name__ == '__main__':
    Marker1 = Send_marker()
    Marker1.run()
