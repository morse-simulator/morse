from morse.builder import *

# Land robot
atrv = ATRV()

gyroscope = Gyroscope()
gyroscope.translate(z = 0.75)
atrv.append(gyroscope)

motion = MotionVW()
atrv.append(motion)

# Add datastream for our robot's components
gyroscope.add_stream("morse.middleware.moos_datastream.MOOS", "morse.middleware.moos.gyroscope.GyroscopeNotifier")
motion.add_stream("morse.middleware.moos_datastream.MOOS", "morse.middleware.moos.motion.MotionReader")

env = Environment('indoors-1/indoor-1')
