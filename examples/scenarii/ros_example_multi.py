from morse.builder import *

# http://www.openrobots.org/morse/doc/latest/user/tutorial.html

def import_atrv(mw):
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
    gyroscope.configure_mw(mw)
    motion.configure_mw(mw)
    return atrv

atrv1 = import_atrv('ros')
atrv1.translate(x=-6)

atrv2 = import_atrv('ros')
atrv2.translate(x=-4)

atrv3 = import_atrv('ros')
atrv3.translate(x=-2)

env = Environment('indoors-1/indoor-1')
env.aim_camera([1.0470, 0, 0.7854])

# rostopic pub /ATRV/Motion_Controller geometry_msgs/Twist -1 [1,0,0] [0,0,1]
# rostopic pub /ATRV.001/Motion_Controller.001 geometry_msgs/Twist -1 [1,0,0] [0,0,1]
# rostopic pub /ATRV.002/Motion_Controller.002 geometry_msgs/Twist -1 [1,0,0] [0,0,1]
