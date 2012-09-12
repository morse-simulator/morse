from morse.builder import *

# http://www.openrobots.org/morse/doc/latest/user/tutorial.html

# Append ATRV robot to the scene
atrv = Robot('atrv')

# Append an actuator
motion = Actuator('v_omega')
motion.translate(z=0.3)
atrv.append(motion)

# Append a Gyroscope sensor
gyroscope = Sensor('gyroscope')
gyroscope.translate(z=0.83)
atrv.append(gyroscope)

# Configuring the middlewares
gyroscope.configure_mw('ros')
motion.configure_mw('ros')

env = Environment('indoors-1/indoor-1')
env.aim_camera([1.0470, 0, 0.7854])
