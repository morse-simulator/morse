from morse.builder import *


""" Cat (Quadrotor) """
cat = Quadrotor()
cat.translate(x=-6.0, z=2.2)
#Cat.rotate(z=-1.5708)

# Waypoint controller (x,y,z, yaw and tolerance (default is 0.2))
waypoint = RotorcraftWaypoint()
cat.append(waypoint)
waypoint.add_stream('socket')

semanticC = SemanticCamera()
semanticC.translate(x=0.3, z=-0.05)
semanticC.rotate(x=+0.2)
cat.append(semanticC)
semanticC.add_stream('socket')
semanticC.properties(Vertical_Flip=False)

catPose = Pose()
cat.append(catPose)
catPose.add_stream('socket')



""" mouse (atrv)"""
mouse = ATRV()
mouse.properties(Object = True, Graspable = False, Label = "MOUSE")
mouse.translate (x=1.0, z=0.2)

keyb = Keyboard()
keyb.properties(Speed=4.0)
mouse.append(keyb)

mousePose = Pose()
mouse.append(mousePose)
mousePose.add_stream('socket')



""" The playground """
env = Environment('land-1/trees')
env.place_camera([10.0, -10.0, 10.0])
env.aim_camera([1.0470, 0, 0.7854])
env.select_display_camera(semanticC)
