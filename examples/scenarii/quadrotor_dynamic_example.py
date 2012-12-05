from morse.builder.morsebuilder import *

import math

bpy.context.scene.game_settings.fps = 120
bpy.context.scene.game_settings.logic_step_max = 5
bpy.context.scene.game_settings.physics_step_max = 5

waypoint_controller = True

# Simple quadrotor with rigid body physics
Quadrotor = Robot('quadrotor_dynamic')
Quadrotor.translate(x= -1.2483, y=1.7043, z=1.8106)
Quadrotor.name = 'mav'

if waypoint_controller:
    motion = Actuator('rotorcraft_waypoint')
    motion.name = 'waypoint'
    motion.configure_mw('ros')
else:
    # simple controller taking RC-like roll/pitch/yaw/thrust input
    motion = Actuator('rotorcraft_attitude')
    motion.name = 'attitude'
    motion.configure_mw('morse.middleware.ros_datastream.rosclass', ['morse.middleware.ros_datastream.ROS', 'read_ctrl_input', 'morse/middleware/ros/read_asctec_ctrl_input'])

Quadrotor.append(motion)

imu = Sensor('imu')
imu.name = 'imu'
# IMU with z-axis down (NED)
imu.rotate(x=math.pi)
imu.configure_mw('ros')
Quadrotor.append(imu)


env = Environment('indoors-1/indoor-1')
env.show_framerate(True)
#env.show_physics(True)

env.create()
