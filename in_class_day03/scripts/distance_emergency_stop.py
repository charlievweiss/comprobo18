#!/usr/bin/env python

"""
This might work
UPDATE: It works
"""

from __future__ import print_function, division #for python2 users
import rospy
# from neato_node.msg import Bump #bump package
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan

class DistanceEmergencyStopNode(object):
    def __init__(self):
        rospy.init_node('emergency_stop')
        rospy.Subscriber('/scan', LaserScan, self.process_scan)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.objdistance = 1.0
        self.linearVel = 0.3

    def process_scan(self, m):
        if m.ranges[0] != 0.0:
            self.objdistance = m.ranges[0]

    def interpret_scan(self):
        if self.objdistance < .5:
            self.linearVel = 0.0

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.interpret_scan()
            self.pub.publish(Twist(linear=Vector3(x=self.linearVel)))
            r.sleep()

if __name__ == '__main__':
    estop = DistanceEmergencyStopNode()
    estop.run()
