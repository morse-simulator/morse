from morse.core import blenderapi
import math


def limit(cont):
    """
    Limit the hand's location to a sphere (radius 0.7) around the shoulder
    """
    ow = cont.owner
    
    # get the suffix of the human to reference the right objects
    suffix = ow.name[-4:] if ow.name[-4] == "." else ""

    limitY = cont.actuators['LimitLocY']
    sobList =  blenderapi.scene().objects
    shoulder = sobList['Shoulder_Empty.R' + suffix]
    human = ow.parent
    
    
    try:
        if human['Manipulate']:
            limitY.min = -math.sqrt(0.7**2 - (shoulder.worldPosition[2] -
                                              ow.worldPosition[2])**2)

            limitY.max = -limitY.min
            cont.activate(limitY)
        
        else:
            cont.deactivate(limitY)
    except ValueError:
        pass
