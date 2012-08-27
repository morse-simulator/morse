from bge import logic

co = logic.getCurrentController()
sens = co.sensors['ManipulateState']
act = co.actuators[0]
objects = logic.getCurrentScene().objects

def add_level():
    if sens.positive and (objects['Human_Camera'].worldPosition == objects['POS_1P_Cam'].worldPosition):
        co.activate(act)
