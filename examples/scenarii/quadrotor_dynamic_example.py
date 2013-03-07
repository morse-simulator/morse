from morse.builder import *
import math

waypoint_controller = True

# must be called before creating component
bpymorse.set_speed(fps = 120, logic_step_max = 5, physics_step_max = 5)

# Simple quadrotor with rigid body physics
quadrotor = Quadrotor()
quadrotor.translate(x= -1.2483, y=1.7043, z=1.8106)
quadrotor.name = 'mav'

if waypoint_controller:
    motion = RotorcraftWaypoint()
    motion.name = 'waypoint'
    motion.add_stream('ros')
else:
    # simple controller taking RC-like roll/pitch/yaw/thrust input
    motion = RotorcraftAttitude()
    motion.name = 'attitude'
    motion.add_stream('ros', 'morse.middleware.ros.read_asctec_ctrl_input.CtrlInputReader')

quadrotor.append(motion)

imu = IMU()
imu.name = 'imu'
# IMU with z-axis down (NED)
imu.rotate(x=math.pi)
imu.add_stream('ros')
quadrotor.append(imu)

env = Environment('indoors-1/indoor-1')
env.show_framerate(True)
#env.show_physics(True)

env.create()
