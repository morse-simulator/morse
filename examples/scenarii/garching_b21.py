from morse.builder.morsebuilder import *
from morse.builder.extensions.pr2extension import PR2

# http://www.openrobots.org/morse/doc/latest/user/tutorial.html

# Append b21 robot to the scene
bender = Robot('b21')
bender.translate(x=6.8, y=3.9, z=0.0)

human = Human()
human.translate(x=2.5, y=1, z=0.0)
#human.rotate(z=-3.0)

Motion_Controller = Actuator('xy_omega')
bender.append(Motion_Controller)

Odometry = Sensor('odometry')
bender.append(Odometry)

Pose_sensor = Sensor('pose')
Pose_sensor.name = 'Pose_sensor'
bender.append(Pose_sensor)

IMU = Sensor('imu')
bender.append(IMU)

Sick = Sensor('sick')
Sick.translate(x=0.275, z=0.252)
bender.append(Sick)
Sick.properties(Visible_arc = False)
Sick.properties(laser_range = 30.0000)
Sick.properties(resolution = 1.0000)
Sick.properties(scan_window = 180.0000)

# Keyboard control
keyboard = Actuator('keyboard')
keyboard.name = 'keyboard_control'
bender.append(keyboard)

# Configuring the middlewares
Pose_sensor.configure_mw('ros')
IMU.configure_mw('ros', ['ROS', 'post_velocity_twist', 'morse/middleware/ros/imu'])
Sick.configure_mw('ros')
Motion_Controller.configure_mw('ros')


# Add passive objects
cornflakes = PassiveObject('props/kitchen_objects.blend', 'Cornflakes')
cornflakes.make_graspable(human_readable_label = 'TUM_Cornflakes')
cornflakes.translate(x=0.5, y=1.9, z=0.9)

fork = PassiveObject('props/kitchen_objects.blend', 'Fork')
fork.make_graspable()
fork.translate(x=0.27, y=2.48, z=0.74)
fork.rotate(z=1.45)

knife = PassiveObject('props/kitchen_objects.blend', 'Knife')
knife.make_graspable()
knife.translate(x=0.27, y=2.42, z=0.74)
knife.rotate(z=1.45)

plate = PassiveObject('props/kitchen_objects.blend', 'Plate')
plate.make_graspable()
plate.translate(x=0.195, y=3.635, z=1.704)
plate.rotate(z=1.45)

#bread = PassiveObject('props/kitchen_objects.blend', 'Bread')
#bread.make_graspable()
#bread.translate(x=0.5, y=1.97, z=0.86)
#bread.rotate(z=1.45)

bowl = PassiveObject('props/kitchen_objects.blend', 'Bowl')
bowl.make_graspable()
bowl.translate(x=0.5, y=1.97, z=0.99)
bowl.rotate(z=1.45)

jam = PassiveObject('props/kitchen_objects.blend', 'Jam')
jam.make_graspable()
jam.translate(x=0.5, y=2.6, z=0.99)
jam.rotate(z=1.45)

nutella = PassiveObject('props/kitchen_objects.blend', 'Nutella')
nutella.make_graspable()
nutella.translate(x=0.5, y=2.8, z=0.99)
nutella.rotate(z=1.45)

# Set scenario
env = Environment('garching_lab/garching_kitchen')
env.aim_camera([1.0470, 0, 0.7854])



