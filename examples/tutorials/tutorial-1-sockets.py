from morse.builder import *

# Land robot
atrv = ATRV()

pose = Pose()
pose.translate(z = 0.75)
atrv.append(pose)

motion = MotionVW()
atrv.append(motion)

# Scene configuration
motion.add_service('socket')
pose.add_service('socket')

env = Environment('indoors-1/indoor-1')
env.set_camera_location([5, -5, 6])
env.set_camera_rotation([1.0470, 0, 0.7854])
