from morse.builder import *

# Land robot
atrv = ATRV()

gyroscope = Gyroscope()
gyroscope.translate(z = 0.75)
atrv.append(gyroscope)

pose = Pose()
pose.translate(z = 0.75)
atrv.append(pose)

gps = GPS()
gps.translate(z = 0.75)
atrv.append(gps)

imu = IMU()
imu.translate(z = 0.75)
atrv.append(imu)

sick = Sick()
sick.translate(x = 0.50, z = 0.75)
atrv.append(sick)

motion = MotionVW()
atrv.append(motion)

# Add datastream for our robot's components
gyroscope.add_stream("moos")
motion.add_stream("moos")
pose.add_stream("moos")
gps.add_stream("moos")
imu.add_stream("moos")
sick.add_stream("moos")

env = Environment('indoors-1/indoor-1', fastmode = True)
env.properties(longitude=43.6, latitude=1.433, altitude=0.)
