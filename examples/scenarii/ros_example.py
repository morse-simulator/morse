from morse.builder import *

# http://www.openrobots.org/morse/doc/latest/user/tutorial.html

# Append ATRV robot to the scene
atrv = ATRV()

# Append an actuator
motion = MotionVW()
motion.translate(z=0.3)
atrv.append(motion)

# Append a Gyroscope sensor
gyroscope = Gyroscope()
gyroscope.translate(z=0.83)
atrv.append(gyroscope)

# Configuring the middlewares
gyroscope.add_stream('ros')
motion.add_stream('ros')

env = Environment('indoors-1/indoor-1')
env.aim_camera([1.0470, 0, 0.7854])
