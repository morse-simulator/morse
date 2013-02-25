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
gyroscope.add_stream("morse.middleware.moos_datastream.MOOS", "morse.middleware.moos.gyroscope.GyroscopeNotifier")
motion.add_stream("morse.middleware.moos_datastream.MOOS", "morse.middleware.moos.motion.MotionReader")
pose.add_stream("morse.middleware.moos_datastream.MOOS", "morse.middleware.moos.pose.PoseNotifier")
gps.add_stream("morse.middleware.moos_datastream.MOOS", "morse.middleware.moos.gps.GPSNotifier")
imu.add_stream("morse.middleware.moos_datastream.MOOS", "morse.middleware.moos.imu.IMUNotifier")
sick.add_stream("morse.middleware.moos_datastream.MOOS", "morse.middleware.moos.sick.LIDARNotifier")

env = Environment('indoors-1/indoor-1')
