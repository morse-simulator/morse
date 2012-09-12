from morse.builder import *

# Land robot
ATRV = Robot('atrv')

Pose = Sensor('pose')
Pose.translate(x=-0.2000, z=0.9000)
ATRV.append(Pose)

Camera = Sensor('video_camera')
Camera.translate(x=0.2000, z=0.9000)
ATRV.append(Camera)

Motion_Controller = Actuator('waypoint')
ATRV.append(Motion_Controller)


# Scene configuration
Motion_Controller.configure_mw('yarp')
Pose.configure_mw('yarp')
Camera.configure_mw('yarp')

env = Environment('indoors-1/indoor-1')
env.aim_camera([1.0470, 0, 0.7854])
