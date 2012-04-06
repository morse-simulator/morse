from bge import logic
import math

co =  logic.getCurrentController()
ow = co.owner
limitY = co.actuators['LimitLocY']
sobList =  logic.getCurrentScene().objects
shoulder = sobList['Shoulder_Empty.R']
human = sobList['Human']



def limit():
    """
    Limit the hand's location to a sphere (radius 0.7) around the shoulder
    """
    try:
        if human['Manipulate']:
            limitY.min = -math.sqrt(0.7**2 - (shoulder.worldPosition[2] -
                                              ow.worldPosition[2])**2)

            limitY.max = -limitY.min
            co.activate(limitY)
        
        else:
            co.deactivate(limitY)
    except ValueError:
        pass
