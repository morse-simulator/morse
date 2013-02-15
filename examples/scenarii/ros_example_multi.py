from morse.builder import *

# http://www.openrobots.org/morse/doc/latest/user/tutorial.html

def import_atrv(mw):
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
    gyroscope.add_stream(mw)
    motion.add_stream(mw)
    return atrv

atrv1 = import_atrv('ros')
atrv1.translate(x=-6)

atrv2 = import_atrv('ros')
atrv2.translate(x=-4)

atrv3 = import_atrv('ros')
atrv3.translate(x=-2)

env = Environment('indoors-1/indoor-1')
env.aim_camera([1.0470, 0, 0.7854])

del env

# rostopic pub -1 /atrv1/motion geometry_msgs/Twist "{linear: {x: .1}, angular: {z: .1}}"
# rostopic pub -1 /atrv2/motion geometry_msgs/Twist "{linear: {x: .1}, angular: {z: .1}}"
# rostopic pub -1 /atrv3/motion geometry_msgs/Twist "{linear: {x: .1}, angular: {z: .1}}"
