from morse.builder import *

# Land robot
atrv = ATRV()

gyroscope = Gyroscope()
gyroscope.translate(z = 0.75)
atrv.append(gyroscope)

motion = MotionVW()
atrv.append(motion)

# Add datastream for our robot's components
gyroscope.add_stream('moos')
motion.add_stream('moos')

env = Environment('indoors-1/indoor-1')
