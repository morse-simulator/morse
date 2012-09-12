from morse.builder import *
from math import pi

human = Human()
human.translate(x = 31.0, y = -12.0, z = 0.0)
human.rotate(x = 0.0, y = 0.0, z = pi)

# passive Objects
plate = PassiveObject('props/kitchen_objects.blend', 'Plate')
plate.setgraspable()
plate.translate(x=-2.9, y=1.89, z=0.92)

fork = PassiveObject('props/kitchen_objects.blend', 'Fork')
fork.setgraspable()
fork.translate(x=-2.9, y=1.69, z=0.91)
fork.rotate(z=pi/2)

knife = PassiveObject('props/kitchen_objects.blend', 'Knife')
knife.setgraspable()
knife.translate(x=-2.9, y=2.09, z=0.91)
knife.rotate(z=pi/2)

bottle = PassiveObject('props/misc_objects.blend', 'Bottle')
bottle.setgraspable()
bottle.translate(x=-3.5, y=1.7, z=1.24)

cup1 = PassiveObject('props/kitchen_objects.blend', 'Cup_Ocher')
cup1.setgraspable()
cup1.translate(x=3.3, y=-5.6, z=1.07)
cup1.rotate(z=0.1)

cup2 = PassiveObject('props/kitchen_objects.blend', 'Cup_Blue')
cup2.setgraspable()
cup2.translate(x=3.4, y=-5.35, z=1.07)
cup2.rotate(z=pi/2)

cup3 = PassiveObject('props/kitchen_objects.blend', 'Cup_Gray')
cup3.setgraspable()
cup3.translate(x=3.5, y=-5.6, z=1.07)

book1 = PassiveObject('props/misc_objects.blend', 'Book_Wine_fat')
book1.setgraspable()
book1.translate(x=0.37, y=5.53, z=0.145)

book2 = PassiveObject('props/misc_objects.blend', 'Book_Blue_fat')
book2.setgraspable()
book2.translate(x=0.46, y=5.55, z=0.165)

book3 = PassiveObject('props/misc_objects.blend', 'Book_Wine_thin')
book3.setgraspable()
book3.translate(x=0.53, y=5.53, z=0.136)
book3.rotate(y=-0.28)

book4 = PassiveObject('props/misc_objects.blend', 'Book_Blue_fat')
book4.setgraspable()
book4.translate(x=0.685, y=5.5, z=0.162)

book5 = PassiveObject('props/misc_objects.blend', 'Book_Wine_thin')
book5.setgraspable()
book5.translate(x=0.34, y=5.53, z=0.5265)

book6 = PassiveObject('props/misc_objects.blend', 'Book_Brown_medium')
book6.setgraspable()
book6.translate(x=0.7, y=5.53, z=0.476)
book6.rotate(y=pi/2)

book6 = PassiveObject('props/misc_objects.blend', 'Book_Brown_thin')
book6.setgraspable()
book6.translate(x=0.87, y=5.5, z=0.49)
book6.rotate(y=-1.29)

book7 = PassiveObject('props/misc_objects.blend', 'Book_Blue_medium')
book7.setgraspable()
book7.translate(x=3.25, y=2.99, z=0.934)
book7.rotate(x=1.879, y=pi/2)

env = Environment('human_tut/tutorial_scene')
