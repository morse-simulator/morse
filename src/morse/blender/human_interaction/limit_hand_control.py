from bge import logic
import math

co =  logic.getCurrentController()
ow = co.owner
limitY = co.actuators['LimitLocY']
sobList =  logic.getCurrentScene().objects
shoulder = sobList['Shoulder_Empty.R']
human = sobList['POS_EMPTY']



def limit():
    """
    Limit the hand's location to a sphere around the shoulder
    """
    try:
        if human['Manipulate'] == True:
            limitY.min = -math.sqrt(0.7**2 - (shoulder.position[2]-ow.position[2])**2)

            limitY.max = -limitY.min
            co.activate(limitY)
        
        else:
            co.deactivate(limitY)
    except ValueError:
        pass
