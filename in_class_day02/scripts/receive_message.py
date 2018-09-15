#!/usr/bin/env python

""" Investigate receiving a message using a callback function """

from __future__ import print_function

from geometry_msgs.msg import PointStamped
import rospy


"""rospy.init_node('receive_message')

def process_point(msg):

    print(msg.header)

rospy.Subscriber("/my_point", PointStamped, process_point)

rospy.spin()"""

class ReceiveMessageNode(object):
    def __init__(self):
        rospy.init_node('receive_message_node')
        rospy.Subscriber('/my_point', PointStamped, self.process_point)

    def process_point(self, m):
        print(m)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = ReceiveMessageNode()
node.run()