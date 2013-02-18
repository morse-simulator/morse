from morse.builder import *


""" Cat (Quadrotor) """
cat = Quadrotor()
cat.name = "CAT"
cat.translate(x=-6.0, z=2.2)
#Cat.rotate(z=-1.5708)

# Waypoint controller (x,y,z, yaw and tolerance (default is 0.2))
waypoint = RotorcraftWaypoint()
waypoint.name = 'waypoint'
cat.append(waypoint)
waypoint.add_stream('socket')

semanticC = SemanticCamera()
semanticC.translate(x=0.3, z=-0.05)
semanticC.rotate(x=-0.2)
semanticC.name = "Camera"
cat.append(semanticC)
semanticC.add_stream('socket')
semanticC.properties(Vertical_Flip=False)

catPose = Pose()
catPose.name = "Cat_Pose"
cat.append(catPose)
catPose.add_stream('socket')

# imu = IMU()
# imu.name = 'imu'
# # IMU with z-axis down (NED)
# imu.rotate(x=math.pi)
# imu.add_stream('ros')
# cat.append(imu)



""" Mouse (atrv)"""
mouse = ATRV()
mouse.name = "MOUSE"
mouse.properties(Object = True, Graspable = False, Label = "MOUSE")
mouse.translate (x=1.0, z=0.2)

keyb = Keyboard()
keyb.properties(Speed=4.0)
mouse.append(keyb)

mousePose = Pose()
mousePose.name = "Mouse_Pose"
mouse.append(mousePose)
mousePose.add_stream('socket')



""" The playground """
env = Environment('land-1/trees')
env.place_camera([10.0, -10.0, 10.0])
env.aim_camera([1.0470, 0, 0.7854])
env.select_display_camera(semanticC)
