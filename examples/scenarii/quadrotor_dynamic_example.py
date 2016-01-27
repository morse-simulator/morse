from morse.builder import *
import math

# choose either 'attitude', 'velocity' or 'waypoint' for control
#control = 'attitude'
control = 'velocity'
#control = 'waypoint'



# Simple quadrotor with rigid body physics
quadrotor = Quadrotor()
quadrotor.translate(x=-1.2483, y=1.7043, z=1.8106)
quadrotor.name = 'mav'

if 'attitude' in control:
    # simple controller taking RC-like roll/pitch/yaw/thrust input
    motion = RotorcraftAttitude()
    motion.name = 'attitude'
    motion.add_stream('ros', 'morse.middleware.ros.read_asctec_ctrl_input.CtrlInputReader')
elif 'velocity' in control:
    motion = RotorcraftVelocity()
    motion.name = 'velocity'
    # read a Twist message
    motion.add_stream('ros')
else:
    motion = RotorcraftWaypoint()
    motion.name = 'waypoint'
    # read a Pose message
    motion.add_stream('ros')

quadrotor.append(motion)

imu = IMU()
imu.name = 'imu'
# IMU with z-axis down (NED)
imu.rotate(x=math.pi)
imu.add_stream('ros')
quadrotor.append(imu)

env = Environment('indoors-1/indoor-1')
env.show_framerate(True)
env.simulator_frequency(fps=60, logic_step_max=5, physics_step_max=5)
#env.show_physics(True)

env.create()
