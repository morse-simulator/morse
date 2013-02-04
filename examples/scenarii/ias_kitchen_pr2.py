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
james.translate(x=0.1, y=2.7, z=0.0)

human = Human()
human.rotate(z=-3.0)

human_pose = Pose()
human.append(human_pose)

# Sensors and Actuators for navigation stack
pr2_posture = PR2Posture()
james.append(pr2_posture)

#semantic_camera = SemanticCamera()
#semantic_camera.translate(x=0.086, y=0, z=1.265)
#james.append(semantic_camera)

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
Pose_sensor.configure_mw('ros')
human_pose.configure_mw('ros')
Sick.configure_mw('ros')
Motion_Controller.configure_mw('ros')
IMU.configure_mw('ros')
pr2_posture.configure_mw('ros', ['ROS', 'post_jointState', 'morse/middleware/ros/pr2_posture'])
#semantic_camera.configure_mw('ros', ['morse.middleware.ros_mw.ROSClass', 'post_lisp_code', 'morse/middleware/ros/semantic_camera'])
#semantic_camera.configure_mw('ros')

#morse.middleware.ros_mw.ROSClass

# Add passive objects
cornflakes = PassiveObject('props/kitchen_objects.blend', 'Cornflakes')
cornflakes.setgraspable()
cornflakes.translate(x=1.37, y=0.5, z=0.9)

fork = PassiveObject('props/kitchen_objects.blend', 'Fork')
fork.setgraspable()
fork.translate(x=1.38, y=0.5, z=0.86)
fork.rotate(z=1.45)

knife = PassiveObject('props/kitchen_objects.blend', 'Knife')
knife.setgraspable()
knife.translate(x=1.39, y=0.5, z=0.86)
knife.rotate(z=1.45)

plate = PassiveObject('props/kitchen_objects.blend', 'Plate')
plate.setgraspable()
plate.translate(x=-1.36, y=2.27, z=0.86)
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
jam.translate(x=1.33, y=0.45, z=0.86)
jam.rotate(z=1.45)

nutella = PassiveObject('props/kitchen_objects.blend', 'Nutella')
nutella.setgraspable()
nutella.translate(x=1.35, y=0.42, z=0.86)
nutella.rotate(z=1.45)

# Set scenario
env = Environment('tum_kitchen/tum_kitchen')
env.aim_camera([1.0470, 0, 0.7854])



