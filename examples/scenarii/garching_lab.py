from morse.builder.morsebuilder import *

# http://www.openrobots.org/morse/doc/latest/user/tutorial.html

# Append b21 robot to the scene
bender = Robot('b21')
bender.translate(x=6.8, y=3.9, z=0.0)

object_tracker = Sensor('object_tracker')
#object_tracker.translate(x=0.086, y=0, z=1.265)
bender.append(object_tracker)
object_tracker.configure_mw('ros', ['morse.middleware.ros_mw.ROSClass', 'post_lisp_code', 'morse/middleware/ros/object_tracker'])

human = Human()
human.translate(x=2.5, y=1, z=0.0)
#human.rotate(z=-3.0)

Pose_sensor = Sensor('pose')
Pose_sensor.name = 'Pose'
human.append(Pose_sensor)

# Configuring the middlewares
Pose_sensor.configure_mw('ros')

# Add passive objects

table = PassiveObject('props/objects.blend','SmallTable')
#table.rotate(z=1.485)
# without object displacement
#table.translate(x=-0.928, y=-0.544, z=0)
table.translate(x=3.11, y=3.151, z=0)

cornflakes = PassiveObject('props/kitchen_objects.blend', 'Cornflakes')
cornflakes.setgraspable()
cornflakes.translate(x=0.5, y=1.9, z=0.9)

fork = PassiveObject('props/kitchen_objects.blend', 'Fork')
fork.setgraspable()
fork.translate(x=0.27, y=2.48, z=0.74)
fork.rotate(z=1.45)

knife = PassiveObject('props/kitchen_objects.blend', 'Knife')
knife.setgraspable()
knife.translate(x=0.27, y=2.42, z=0.74)
knife.rotate(z=1.45)

plate = PassiveObject('props/kitchen_objects.blend', 'Plate')
plate.setgraspable()
plate.translate(x=0.195, y=3.635, z=1.704)
plate.rotate(z=1.45)

#bread = PassiveObject('props/kitchen_objects.blend', 'Bread')
#bread.setgraspable()
#bread.translate(x=0.5, y=1.97, z=0.86)
#bread.rotate(z=1.45)

bowl = PassiveObject('props/kitchen_objects.blend', 'Bowl')
bowl.setgraspable()
bowl.translate(x=0.5, y=1.97, z=0.99)
bowl.rotate(z=1.45)

jam = PassiveObject('props/kitchen_objects.blend', 'Jam')
jam.setgraspable()
jam.translate(x=0.5, y=2.6, z=0.99)
jam.rotate(z=1.45)

nutella = PassiveObject('props/kitchen_objects.blend', 'Nutella')
nutella.setgraspable()
nutella.translate(x=0.5, y=2.8, z=0.99)
nutella.rotate(z=1.45)

# Set scenario
env = Environment('garching_lab/garching_kitchen')
env.aim_camera([1.0470, 0, 0.7854])



