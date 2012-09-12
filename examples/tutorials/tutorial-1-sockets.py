from morse.builder import *

# Land robot
ATRV = Robot('atrv')

Pose = Sensor('pose')
Pose.translate(x=-0.2000, z=0.9000)
ATRV.append(Pose)

Motion_Controller = Actuator('v_omega')
ATRV.append(Motion_Controller)


# Scene configuration
Motion_Controller.configure_service('socket')
Pose.configure_service('socket')

env = Environment('indoors-1/indoor-1')
env.place_camera([5, -5, 6])
env.aim_camera([1.0470, 0, 0.7854])
