from morse.builder.morsebuilder import *

# http://www.openrobots.org/morse/doc/latest/user/tutorial.html

# Append ATRV robot to the scene
atrv = Robot('atrv')

# Append an actuator
motion = Controller('morse_vw_control')
motion.translate(z=0.3)
atrv.append(motion)

# Append a Gyroscope sensor
gyroscope = Sensor('morse_gyroscope')
gyroscope.translate(z=0.83)
atrv.append(gyroscope)

# Insert the middleware object
ros = Middleware('ros_empty')

# Configuring the middlewares
ros.configure(gyroscope)
ros.configure(motion)

