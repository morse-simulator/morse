from morse.builder.morsebuilder import *
from morse.builder.pr2extension import PR2

# http://www.openrobots.org/morse/doc/latest/user/tutorial.html

# Append ATRV robot to the scene
james = PR2()
james.configure_service('ros')
james.head.configure_overlay('ros', 'morse.middleware.ros.overlays.pr2.PR2')

motion = Actuator('v_omega')
motion.translate(z=0.3)
james.append(motion)

# Keyboard control
keyboard = Actuator('keyboard')
james.append(keyboard)

# Append a Gyroscope sensor
gyroscope = Sensor('gyroscope')
gyroscope.translate(z=0.83)
james.append(gyroscope)

# Configuring the middlewares
#gyroscope.configure_mw('ros')
#motion.configure_mw('ros')

env = Environment('indoors-1/indoor-1')
env.aim_camera([1.0470, 0, 0.7854])
