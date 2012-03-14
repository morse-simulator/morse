from morse.builder.morsebuilder import *
from morse.builder.extensions.pr2extension import PR2

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
pr2_posture = Sensor('pr2_posture')
james.append(pr2_posture)

Motion_Controller = Actuator('xy_omega')
james.append(Motion_Controller)

Odometry = Sensor('odometry')
james.append(Odometry)

Pose_sensor = Sensor('pose')
Pose_sensor.name = 'Pose_sensor'
james.append(Pose_sensor)

IMU = Sensor('imu')
james.append(IMU)

Sick = Sensor('sick')
Sick.translate(x=0.275, z=0.252)
james.append(Sick)
Sick.properties(Visible_arc = False)
Sick.properties(laser_range = 30.0000)
Sick.properties(resolution = 1.0000)
Sick.properties(scan_window = 180.0000)

# Keyboard control
keyboard = Actuator('keyboard')
keyboard.name = 'keyboard_control'
james.append(keyboard)

# Configuring the middlewares
Pose_sensor.configure_mw('ros')
Sick.configure_mw('ros')
Motion_Controller.configure_mw('ros')
IMU.configure_mw('ros')
pr2_posture.configure_mw('ros', ['ROS', 'post_jointState', 'morse/middleware/ros/pr2_posture'])

# Add passive objects
cornflakes = PassiveObject('props/kitchen_objects.blend', 'Cornflakes')
cornflakes.setgraspable()
cornflakes.translate(x=0.5, y=1.67, z=0.9)

fork = PassiveObject('props/kitchen_objects.blend', 'Fork')
fork.setgraspable()
fork.translate(x=0.5, y=1.87, z=0.86)
fork.rotate(z=1.45)

knife = PassiveObject('props/kitchen_objects.blend', 'Knife')
knife.setgraspable()
knife.translate(x=0.5, y=1.97, z=0.86)
knife.rotate(z=1.45)

plate = PassiveObject('props/kitchen_objects.blend', 'Plate')
plate.setgraspable()
plate.translate(x=0.5, y=1.97, z=0.86)
plate.rotate(z=1.45)

#bread = PassiveObject('props/kitchen_objects.blend', 'Bread')
#bread.setgraspable()
#bread.translate(x=0.5, y=1.97, z=0.86)
#bread.rotate(z=1.45)

bowl = PassiveObject('props/kitchen_objects.blend', 'Bowl')
bowl.setgraspable()
bowl.translate(x=0.5, y=1.97, z=0.86)
bowl.rotate(z=1.45)

jam = PassiveObject('props/kitchen_objects.blend', 'Jam')
jam.setgraspable()
jam.translate(x=0.5, y=1.97, z=0.86)
jam.rotate(z=1.45)

nutella = PassiveObject('props/kitchen_objects.blend', 'Nutella')
nutella.setgraspable()
nutella.translate(x=0.5, y=1.97, z=0.86)
nutella.rotate(z=1.45)

# Set scenario
env = Environment('tum_kitchen/tum_kitchen')
env.aim_camera([1.0470, 0, 0.7854])



