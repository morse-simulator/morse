import logging; logger = logging.getLogger("morse." + __name__)
import os

from morse.core import blenderapi

""" Generic Python Module to be called by all MORSE components.
    It will locate the calling object in the dictionary,
    retrieve the stored instance and call its 'action' method. """

def robot_action(contr):
    """ Call the 'action' method of the correct robot. """
    scene = blenderapi.scene()
    # if current scene is not the main MORSE logic scene, pass
    if not scene or scene.name != 'S.MORSE_LOGIC':
        return
    # Do nothing if morse has not been properly initialised
    simu = blenderapi.persistantstorage()
    if "morse_initialised" not in simu or not simu.morse_initialised:
        return

    # Execute only when the sensor is really activated
    if contr.sensors[0].positive:
        obj = contr.owner

        # Do nothing if the component was not initialised.
        # Should be the case for external robots and components
        robot_object = simu.robotDict.get(obj, None)
        if robot_object:
            robot_object.action()

def component_action(contr):
    """ Call the 'action' method of the correct component. """
    scene = blenderapi.scene()
    # if current scene is not the main MORSE logic scene, pass
    if not scene or scene.name != 'S.MORSE_LOGIC':
        return
    # Do nothing if morse has not been properly initialised
    simu = blenderapi.persistantstorage()
    if "morse_initialised" not in simu or not simu.morse_initialised:
        return

    # Execute only when the sensor is really activated
    if contr.sensors[0].positive:
        obj = contr.owner

        # Do nothing if the component was not initialised.
        # Should be the case for external robots and components
        cmpt_object = simu.componentDict.get(obj.name, None)
        if cmpt_object:
            cmpt_object.action()

def sensor_action(contr):
    """ Call the 'action' method of the correct sensor. """
    component_action(contr)

def actuator_action(contr):
    """ Call the 'action' method of the correct actuator. """
    component_action(contr)
