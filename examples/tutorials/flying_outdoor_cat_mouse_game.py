from morse.builder import *

#import math


""" Cat (Quadrotor) """
Cat = Quadrotor()
Cat.name = "CAT"
Cat.translate(x=-6.0, z=2.2)
#Cat.rotate(z=-1.5708)

# Waypoint controller (x,y,z, yaw and tolerance (default is 0.2))
waypoint = RotorcraftWaypoint()
waypoint.name = 'waypoint'
Cat.append(waypoint)
waypoint.add_stream('socket')

SemanticC = SemanticCamera()
SemanticC.translate(x=0.3, z=-0.05)
SemanticC.rotate(x=-0.2)
SemanticC.name = "Camera"
Cat.append(SemanticC)
SemanticC.add_stream('socket')

CatPose = Pose()
CatPose.name = "Cat_Pose"
Cat.append(CatPose)
CatPose.add_stream('socket')

# imu = IMU()
# imu.name = 'imu'
# # IMU with z-axis down (NED)
# imu.rotate(x=math.pi)
# imu.add_stream('ros')
# Quadrotor.append(imu)


""" Mouse (atrv)"""
Mouse = ATRV()
Mouse.name = "MOUSE"
Mouse.properties(Object = True, Graspable = False, Label = "MOUSE")
Mouse.translate (x=1.0, z=0.2)

Keyb = Keyboard()
Keyb.properties(Speed=4.0)
Mouse.append(Keyb)

MousePose = Pose()
MousePose.name = "Mouse_Pose"
Mouse.append(MousePose)
MousePose.add_stream('socket')


""" The playground """
env = Environment('land-1/trees')
env.place_camera([10.0, -10.0, 10.0])
env.aim_camera([1.0470, 0, 0.7854])
env.select_display_camera(SemanticC)




