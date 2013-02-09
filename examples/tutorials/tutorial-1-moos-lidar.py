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
gyroscope.add_stream("morse.middleware.moos_datastream.MOOS", "post_gyroscope", "morse/middleware/moos/gyroscope")
pose.add_stream("morse.middleware.moos_datastream.MOOS", "post_pose", "morse/middleware/moos/pose")
gps.add_stream("morse.middleware.moos_datastream.MOOS", "post_gps", "morse/middleware/moos/gps")
imu.add_stream("morse.middleware.moos_datastream.MOOS", "post_imu", "morse/middleware/moos/imu")
sick.add_stream("morse.middleware.moos_datastream.MOOS", "post_2DLaserScan", "morse/middleware/moos/sick")
motion.add_stream("morse.middleware.moos_datastream.MOOS", "read_message")

env = Environment('indoors-1/indoor-1')
