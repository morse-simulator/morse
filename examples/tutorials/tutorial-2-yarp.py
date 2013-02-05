from morse.builder import *

# Land robot
ATRV = ATRV()

Pose = Pose()
Pose.translate(x=-0.2000, z=0.9000)
ATRV.append(Pose)

Camera = VideoCamera()
Camera.translate(x=0.2000, z=0.9000)
ATRV.append(Camera)

Motion_Controller = Waypoint()
ATRV.append(Motion_Controller)


# Scene configuration
Motion_Controller.add_stream('yarp')
Pose.add_stream('yarp')
Camera.add_stream('yarp')

env = Environment('indoors-1/indoor-1')
env.aim_camera([1.0470, 0, 0.7854])
