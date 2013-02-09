from morse.builder import *

# Land robot
atrv = ATRV()

pose = Pose()
pose.translate(z = 0.75)
atrv.append(pose)

motion = MotionVW()
atrv.append(motion)

# Scene configuration
motion.configure_service('socket')
pose.configure_service('socket')

env = Environment('indoors-1/indoor-1')
env.place_camera([5, -5, 6])
env.aim_camera([1.0470, 0, 0.7854])
