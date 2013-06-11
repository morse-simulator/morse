""" MORSE Kinect example
"""
from morse.builder import *

robot = ATRV()

keyboard = Keyboard()
keyboard.properties(Speed=3)
robot.append(keyboard)

odometry = Odometry()
odometry.add_stream('ros')
robot.append(odometry)

kinect = Kinect()
kinect.add_stream('ros', frame_id='kinect')
robot.append(kinect)

env = Environment('outdoors')

