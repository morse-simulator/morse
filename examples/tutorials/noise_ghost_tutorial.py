from morse.builder import *

robot = ATRV()
pose = Pose()
robot.append(pose)
pose.add_stream('socket')
keyboard = Keyboard()
robot.append(keyboard)
pose.alter('Noise', pos_std=.5, rot_std=0, _2D='True')

ghost = ATRV()
ghost.make_ghost()
teleport = Teleport()
ghost.append(teleport)
ghost.add_default_interface('socket')

env = Environment("land-1/trees")
env.create()