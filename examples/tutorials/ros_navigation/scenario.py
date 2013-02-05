from morse.builder import *

# Append PR2 robot to the scene
james = BasePR2()
james.translate(x=2.5, y=3.2, z=0.0)

# Sensors and Actuators for navigation stack
#pr2_posture = PR2Posture()
#james.append(pr2_posture)
#pr2_posture.add_stream('ros')

motion_controller = MotionXYW()
james.append(motion_controller)
motion_controller.add_stream('ros')

odometry = Odometry()
james.append(odometry)
odometry.add_stream('ros')

sick = Sick()
sick.translate(x=0.275, z=0.252)
james.append(sick)
sick.properties(Visible_arc=False)
sick.properties(laser_range=30.0000)
sick.properties(resolution=1.0000)
sick.properties(scan_window=180.0000)
sick.add_stream('ros')

# Keyboard control
keyboard = Keyboard()
keyboard.name = 'keyboard_control'
james.append(keyboard)

# Set scenario
env = Environment('tum_kitchen/tum_kitchen')
env.aim_camera([1.0470, 0, 0.7854])
