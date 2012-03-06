from morse.builder.morsebuilder import *
from morse.builder.extensions.pr2extension import PR2

# Append ATRV robot to the scene
james = PR2()
james.translate(x=2.5, y=3.2, z=0.0)

# Sensors and Actuators for navigation stack
pr2_posture = Sensor('pr2_posture')
james.append(pr2_posture)
pr2_posture.configure_mw('ros')

Motion_Controller = Actuator('xy_omega')
james.append(Motion_Controller)
Motion_Controller.configure_mw('ros')

imu = Sensor('imu')
james.append(imu)
imu.configure_mw('ros')

Sick = Sensor('sick')
Sick.translate(x=0.275, z=0.252)
james.append(Sick)
Sick.properties(Visible_arc = False)
Sick.properties(laser_range = 30.0000)
Sick.properties(resolution = 1.0000)
Sick.properties(scan_window = 180.0000)
Sick.configure_mw('ros')

# Keyboard control
keyboard = Actuator('keyboard')
keyboard.name = 'keyboard_control'
james.append(keyboard)

# Set scenario
env = Environment('tum_kitchen/tum_kitchen')
env.aim_camera([1.0470, 0, 0.7854])

