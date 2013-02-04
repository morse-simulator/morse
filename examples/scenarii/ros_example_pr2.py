from morse.builder import *
from morse.builder.robots.pr2 import PR2

# http://www.openrobots.org/morse/doc/latest/user/tutorial.html

# Append ATRV robot to the scene
james = PR2()
james.configure_service('ros')
james.head.configure_overlay('ros', 'morse.middleware.ros.overlays.pr2.PR2')
james.l_arm.configure_overlay('ros', 'morse.middleware.ros.overlays.pr2.PR2')
james.r_arm.configure_overlay('ros', 'morse.middleware.ros.overlays.pr2.PR2')
james.torso_lift.configure_overlay('ros', 'morse.middleware.ros.overlays.pr2.PR2')
james.translate(x=2.5, y=3.2, z=0.0)

human = Human()
human.translate(x=2.5, y=0, z=0.0)
#human.rotate(z=-3.0)

# Sensors and Actuators for navigation stack
pr2_posture = PR2Posture()
james.append(pr2_posture)

Motion_Controller = MotionXYW()
james.append(Motion_Controller)

Odometry = Odometry()
james.append(Odometry)

Pose_sensor = Pose()
Pose_sensor.name = 'Pose_sensor'
james.append(Pose_sensor)

IMU = IMU()
james.append(IMU)

Sick = Sick()
Sick.translate(x=0.275, z=0.252)
james.append(Sick)
Sick.properties(Visible_arc = False)
Sick.properties(laser_range = 30.0000)
Sick.properties(resolution = 1.0000)
Sick.properties(scan_window = 180.0000)

# Keyboard control
keyboard = Keyboard()
keyboard.name = 'keyboard_control'
james.append(keyboard)

# Configuring the middlewares
Pose_sensor.add_stream('ros')
Sick.add_stream('ros')
Motion_Controller.add_stream('ros')
IMU.add_stream('ros')
pr2_posture.add_stream('ros', 'morse.middleware.ros.jointstate.JointStatePR2Publisher')

# Add passive objects
cornflakes = PassiveObject('props/kitchen_objects', 'Cornflakes')
cornflakes.setgraspable()
cornflakes.translate(x=0.5, y=1.67, z=0.9)

fork = PassiveObject('props/kitchen_objects', 'Fork')
fork.setgraspable()
fork.translate(x=0.5, y=1.87, z=0.86)
fork.rotate(z=1.45)

knife = PassiveObject('props/kitchen_objects', 'Knife')
knife.setgraspable()
knife.translate(x=0.5, y=1.97, z=0.86)
knife.rotate(z=1.45)

plate = PassiveObject('props/kitchen_objects', 'Plate')
plate.setgraspable()
plate.translate(x=0.5, y=1.97, z=0.86)
plate.rotate(z=1.45)

#bread = PassiveObject('props/kitchen_objects', 'Bread')
#bread.setgraspable()
#bread.translate(x=0.5, y=1.97, z=0.86)
#bread.rotate(z=1.45)

bowl = PassiveObject('props/kitchen_objects', 'Bowl')
bowl.setgraspable()
bowl.translate(x=0.5, y=1.97, z=0.86)
bowl.rotate(z=1.45)

jam = PassiveObject('props/kitchen_objects', 'Jam')
jam.setgraspable()
jam.translate(x=0.5, y=1.97, z=0.86)
jam.rotate(z=1.45)

nutella = PassiveObject('props/kitchen_objects', 'Nutella')
nutella.setgraspable()
nutella.translate(x=0.5, y=1.97, z=0.86)
nutella.rotate(z=1.45)

# Set scenario
env = Environment('tum_kitchen/tum_kitchen')
env.aim_camera([1.0470, 0, 0.7854])
