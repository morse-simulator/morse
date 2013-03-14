from morse.builder import *

from math import pi

""" Cat (Quadrotor) """
cat = Quadrotor()
cat.translate(x=-7.0, z=1.0)
cat.rotate(z=pi/3)

# Waypoint controller (x,y,z, yaw and tolerance (default is 0.2))
waypoint = RotorcraftWaypoint()
cat.append(waypoint)
waypoint.add_stream('socket')

semanticC = SemanticCamera()
semanticC.translate(x=0.3, z=-0.05)
semanticC.rotate(x=+0.2)
cat.append(semanticC)
semanticC.properties(Vertical_Flip=False)

catPose = Pose()
cat.append(catPose)
catPose.add_stream('socket')


""" mouse (atrv)"""
mouse = ATRV()
mouse.translate (x=-4.0,y=6.5, z=0.1)
mouse.rotate(z=0.70*pi)

keyb = Keyboard()
keyb.properties(Speed=2.0)
mouse.append(keyb)

mousePose = Pose()
mouse.append(mousePose)
mousePose.add_stream('socket')


""" The playground """
env = Environment('outdoors')
env.place_camera([10.0, -10.0, 10.0])
env.aim_camera([1.0470, 0, 0.7854])
env.select_display_camera(semanticC)
