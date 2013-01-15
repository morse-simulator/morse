from morse.builder import *
from morse.builder.robots.pr2 import PR2

# Append PR2 robot to the scene
james = PR2()
james.translate(x=2.5, y=3.2, z=0.0)

# Sensors and Actuators for navigation stack
pr2_posture = Sensor('pr2_posture')
james.append(pr2_posture)
pr2_posture.add_stream('ros')

motion_controller = Actuator('xy_omega')
james.append(motion_controller)
motion_controller.add_stream('ros')

odometry = Sensor('odometry')
james.append(odometry)
odometry.add_stream('ros')

Sick = Sensor('sick')
Sick.translate(x=0.275, z=0.252)
james.append(Sick)
Sick.properties(Visible_arc=False)
Sick.properties(laser_range=30.0000)
Sick.properties(resolution=1.0000)
Sick.properties(scan_window=180.0000)
Sick.add_stream('ros')

# Keyboard control
keyboard = Actuator('keyboard')
keyboard.name = 'keyboard_control'
james.append(keyboard)

# Set scenario
env = Environment('tum_kitchen/tum_kitchen')
env.aim_camera([1.0470, 0, 0.7854])

