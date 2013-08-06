""" MORSE Joystick example """
from morse.builder import *

robot = ATRV()

joystick = Joystick()
joystick.properties(Speed=3)
robot.append(joystick)

env = Environment('outdoors')
