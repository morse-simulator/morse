from morse.builder import *

# Land robot
atrv = ATRV()

pose = Pose()
pose.translate(x=-0.2000, z=0.9000)
atrv.append(pose)

camera = VideoCamera()
camera.translate(x=0.2000, z=0.9000)
atrv.append(camera)

motion = Waypoint()
atrv.append(motion)

# Scene configuration
motion.add_stream('yarp')
pose.add_stream('yarp')
camera.add_stream('yarp')

env = Environment('indoors-1/indoor-1')
env.aim_camera([1.0470, 0, 0.7854])
