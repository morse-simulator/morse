from morse.builder import *

# Append ATRV robot to the scene
atrv = ATRV()

# Append an actuator
motion = MotionVW()
motion.translate(z=0.3)
atrv.append(motion)

# Append an odometry sensor
odometry = Odometry()
odometry.translate(x=-0.1, z=0.83)
atrv.append(odometry)

# Append a proximity sensor
proximity = Proximity()
proximity.translate(x=-0.2, z=0.83)
atrv.append(proximity)

# Append a Pose sensor (GPS + Gyroscope)
pose = Pose()
pose.translate(x=0.2,z=0.83)
atrv.append(pose)

# Append a sick laser
sick = Sick()
sick.translate(x=0.18,z=0.94)
atrv.append(sick)
sick.properties(resolution = 1)
sick.properties(laser_range = 5.0)

# Append a camera
cam = VideoCamera()
cam.translate(x=0.3,z=1.1)
atrv.append(cam)
cam.properties(cam_width = 128, cam_height = 128)

# Configure the middlewares
motion.add_stream('yarp')
odometry.add_stream('yarp')
proximity.add_stream('yarp')
pose.add_stream('yarp')
sick.add_stream('yarp')
cam.add_stream('yarp')

# Configure the middlewares
motion.add_stream('socket')
pose.add_stream('socket')

env = Environment('laas/grande_salle')
env.aim_camera([1.0470, 0, 0.7854])
