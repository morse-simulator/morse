#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist

cmd = rospy.Publisher("/atrv/motion", Twist)
motion = Twist()
def callback(msg):
    position = msg.pose.position
    if position.x < 1:
        motion.linear.x = +0.5
    if position.x > 2:
        motion.linear.x = -0.5
    cmd.publish(motion)

rospy.init_node("rostuto1")
rospy.Subscriber("/atrv/pose", PoseStamped, callback)
rospy.spin() # this will block untill you hit Ctrl+C

