""" MORSE Collision example """
from morse.builder import *

dala = ATRV()
dala.properties(obstacle=1)

robot = ATRV()

keyboard = Keyboard()
keyboard.properties(Speed=3)
robot.append(keyboard)

collision = Collision()
collision.properties(collision_property="obstacle")
collision.add_stream('socket')
collision.translate(x = 0.7, z = 0.2)
robot.append(collision)
robot.translate(x = -2.0)

env = Environment('outdoors')
