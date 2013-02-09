from morse.builder import *

# Land robot
atrv = ATRV()

pose = Pose()
pose.translate(z = 0.75)
atrv.append(pose)

motion = MotionVW()
atrv.append(motion)

# Add default interface for our robot's components
atrv.add_default_interface('ros')

env = Environment('indoors-1/indoor-1')
