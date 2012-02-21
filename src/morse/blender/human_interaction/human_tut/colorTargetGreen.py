from bge import logic

objects = logic.getCurrentScene().objects
cube = objects['CubeSelect']
door = objects['Shelf_UpperDoor.R']

co = logic.getCurrentController()
grab = co.sensors['GrabState']
layDown = co.sensors['OpenDoorState']

def color():
    if grab.positive:
        cube.color = [0.0, 1.0, 0.0, 1.0]
    elif layDown.positive:
        cube.color = [0.3, 0.3, 0.3, 1.0]
        door.color = [0.0, 1.0, 0.0, 1.0]
    