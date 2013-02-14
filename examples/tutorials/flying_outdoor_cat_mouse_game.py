from morse.builder import *

#import math


""" Cat (Quadrotor) """
Cat = Robot('quadrotor_dynamic')
Cat.name = "CAT"
Cat.translate(x=-6.0, z=2.2)
#Cat.rotate(z=-1.5708)

# Waypoint controller (x,y,z, yaw and tolerance (default is 0.2))
waypoint = Actuator('rotorcraft_waypoint')
waypoint.name = 'waypoint'
Cat.append(waypoint)
waypoint.configure_mw('socket')

SemanticC = Sensor('semantic_camera')
SemanticC.translate(x=0.3, z=-0.05)
SemanticC.rotate(x=-0.2)
SemanticC.name = "Camera"
Cat.append(SemanticC)
SemanticC.configure_mw('socket')

CatPose = Sensor('pose')
CatPose.name = "Cat_Pose"
Cat.append(CatPose)
CatPose.configure_mw('socket')

# imu = Sensor('imu')
# imu.name = 'imu'
# # IMU with z-axis down (NED)
# imu.rotate(x=math.pi)
# imu.configure_mw('ros')
# Quadrotor.append(imu)


""" Mouse (atrv)"""
Mouse = Robot('atrv')
Mouse.name = "MOUSE"
Mouse.properties(Object = True, Graspable = False, Label = "MOUSE")
Mouse.translate (x=1.0, z=0.2)

Keyb = Actuator('keyboard')
Keyb.properties(Speed=4.0)
Mouse.append(Keyb)

MousePose = Sensor('pose')
MousePose.name = "Mouse_Pose"
Mouse.append(MousePose)
MousePose.configure_mw('socket')


""" The playground """
env = Environment('land-1/trees')
env.place_camera([10.0, -10.0, 10.0])
env.aim_camera([1.0470, 0, 0.7854])
env.select_display_camera(SemanticC)




