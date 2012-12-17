from morse.core import blenderapi

co = blenderapi.controller()
collision = co.sensors['Collision']
objects = blenderapi.scene().objects
human = objects['POS_EMPTY']

def place(): 
    if collision.positive:
        human.worldPosition = (30, -12, 0)
