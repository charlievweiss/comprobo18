#!/usr/bin/env python
""" This script explores publishing ROS messages in ROS using Python"""

from __future__ import print_function
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Header
from geometry_msgs.msg import Point
import rospy
from time import sleep

""" Non-object oriented: 
rospy.init_node('test_message') #initialize ourselves with roscore
my_header = Header(stamp=rospy.Time.now(), frame_id="odom")
my_point = Point(1.0,2.0,0.0)

my_point_stamped = PointStamped(header=my_header, point=my_point)

# alternately, in one line:
# my_point_stamped = PointStamped(header=Header(stamp=rospy.Time.now(), frame_id="odom"), point=Point(1.0, 2.0, 0.0))

publisher = rospy.Publisher('my_point', PointStamped, queue_size=10)
r = rospy.Rate(2)
while not rospy.is_shutdown():
	my_point_stamped.header.stamp = rospy.Time.now() #update timestamp
	publisher.publish(my_point_stamped)
	r.sleep() """

class TestMessageNode(object):
	""" this node publishes a message at 2 Hz """
	def __init__(self):
		rospy.init_node("test_node")
		self.pub = rospy.Publisher('/my_point', PointStamped, queue_size=10)

	def run(self):
		r = rospy.Rate(2)
		while not rospy.is_shutdown():
			my_header = Header(stamp=rospy.Time.now(), frame_id="base_link")
			my_point = Point(x=1.0, y=2.0)
			m = PointStamped(header=my_header, point=my_point)
			self.pub.publish(m)
			r.sleep()

if __name__ == '__main__':
	node = TestMessageNode()
	node.run()