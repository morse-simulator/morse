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
james.translate(x=-3.2, y=2.7, z=0.0)

human = Human()
human.rotate(z=-3.0)

human_pose = Sensor('pose')
human.append(human_pose)

# Sensors and Actuators for navigation stack
pr2_posture = Sensor('pr2_posture')
james.append(pr2_posture)

semantic_camera = Sensor('semantic_camera')
semantic_camera.translate(x=0.086, y=0, z=1.265)
james.append(semantic_camera)

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
human_pose.configure_mw('ros')
Sick.configure_mw('ros')
Motion_Controller.configure_mw('ros')
IMU.configure_mw('ros')
pr2_posture.configure_mw('ros', ['ROS', 'post_jointState', 'morse/middleware/ros/pr2_posture'])
semantic_camera.configure_mw('ros', ['morse.middleware.ros_mw.ROSClass', 'post_lisp_code', 'morse/middleware/ros/semantic_camera'])
#semantic_camera.configure_mw('ros')

#morse.middleware.ros_mw.ROSClass

# Add passive objects

table = PassiveObject('props/objects.blend','SmallTable')
table.rotate(z=1.485)
# without object displacement
#table.translate(x=-0.928, y=-0.544, z=0)
table.translate(x=-0.313, y=-1.237, z=0)

placemat = PassiveObject('props/kitchen_objects.blend', 'Placemat')
placemat.setgraspable()
placemat.translate(x=1.248, y=0.404, z=0.9)

napkin = PassiveObject('props/kitchen_objects.blend', 'Napkin')
napkin.setgraspable()
napkin.translate(x=1.222, y=0.40, z=0.87)

cup = PassiveObject('props/kitchen_objects.blend', 'Cup_Gray')
cup.setgraspable()
cup.translate(x=-1.196, y=1.177, z=0.95)

cornflakes = PassiveObject('props/kitchen_objects.blend', 'Cornflakes')
cornflakes.setgraspable()
cornflakes.translate(x=-1.570, y=1.477,  z=1.060)

spoon = PassiveObject('props/kitchen_objects.blend', 'Spoon')
spoon.setgraspable()
spoon.translate(x=1.279, y=1.060, z=0.86)
spoon.rotate(z=1.45)

fork = PassiveObject('props/kitchen_objects.blend', 'Fork')
fork.setgraspable()
fork.translate(x=1.279, y=1.117, z=0.86)
fork.rotate(z=1.45)

knife = PassiveObject('props/kitchen_objects.blend', 'Knife')
knife.setgraspable()
knife.translate(x=1.279, y=1.189, z=0.86)
knife.rotate(z=1.45)

plate = PassiveObject('props/kitchen_objects.blend', 'Plate')
plate.setgraspable()
plate.translate(x=-1.210, y=1.556, z=0.86)
plate.rotate(z=1.45)

#bread = PassiveObject('props/kitchen_objects.blend', 'Bread')
#bread.setgraspable()
#bread.translate(x=0.5, y=1.97, z=0.86)
#bread.rotate(z=1.45)

bowl = PassiveObject('props/kitchen_objects.blend', 'Bowl')
bowl.setgraspable()
bowl.translate(x=-1.38, y=2.30, z=0.86)
bowl.rotate(z=1.45)

jam = PassiveObject('props/kitchen_objects.blend', 'Jam')
jam.setgraspable()
jam.translate(x=-1.642, y=1.371, z=0.9)
jam.rotate(z=1.45)

nutella = PassiveObject('props/kitchen_objects.blend', 'Nutella')
nutella.setgraspable()
nutella.translate(x=-1.594, y=1.258, z=0.86)
nutella.rotate(z=1.45)

bread = PassiveObject('props/kitchen_objects.blend', 'Bread')
bread.setgraspable()
bread.translate(x=-1.699, y=1.636, z=0.86)
bread.rotate(z=1.45)

# Set scenario
env = Environment('tum_kitchen/tum_kitchen')
env.aim_camera([1.0470, 0, 0.7854])



