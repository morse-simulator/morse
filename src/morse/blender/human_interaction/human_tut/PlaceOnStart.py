from bge import logic

co = logic.getCurrentController()
collision = co.sensors['Collision']
objects = logic.getCurrentScene().objects
human = objects['POS_EMPTY']

def place(): 
    if collision.positive:
        human.worldPosition = (30, -12, 0)