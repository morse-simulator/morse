import logging; logger = logging.getLogger("morse." + __name__)
import bge
from morse.core.services import service
from morse.core import status
from morse.blender.main import reset_objects as main_reset, close_all as main_close, quit as main_terminate
from morse.core.exceptions import *

@service(component = "simulation")
def list_robots():
    """ Return a list of the robots in the current scenario

    Uses the list generated during the initialisation of the scenario
    """
    return [obj.name for obj in bge.logic.robotDict.keys()]

@service(component = "simulation")
def reset_objects():
    """ Restore all simulation objects to their original position

    Upon receiving the request using sockets, call the
    'reset_objects' function located in morse/blender/main.py
    """
    contr = bge.logic.getCurrentController()
    main_reset(contr)
    return "Objects restored to initial position"

@service(component = "simulation")
def quit():
    """ Cleanly quit the simulation
    """
    contr = bge.logic.getCurrentController()
    main_close(contr)
    main_terminate(contr)
    
@service(component = "simulation")
def terminate():
    """ Terminate the simulation (no finalization done!)
    """
    contr = bge.logic.getCurrentController()
    main_terminate(contr)

@service(component = "simulation")
def activate(component_name):
    """ Enable the functionality of the component specified
    """
    try:
        bge.logic.componentDict[component_name]._active = True
    except KeyError as detail:
        logger.warn("Component %s not found. Can't activate" % detail)
        raise MorseRPCTypeError("Component %s not found. Can't activate" % detail)

@service(component = "simulation")
def deactivate(component_name):
    """ Stop the specified component from calling its default_action method
    """
    try:
        bge.logic.componentDict[component_name]._active = False
    except KeyError as detail:
        logger.warn("Component %s not found. Can't deactivate" % detail)
        raise MorseRPCTypeError("Component %s not found. Can't deactivate" % detail)
        
@service(component = "simulation")
def suspend_dynamics():
    """ Suspends physics for all object in the scene.
    """

    scene = bge.logic.getCurrentScene()
    for object in scene.objects:
        object.suspendDynamics()
        
    return "Physics is suspended"
        
@service(component = "simulation")
def restore_dynamics():
    """ Resumes physics for all object in the scene.
    """

    scene = bge.logic.getCurrentScene()
    for object in scene.objects:
        object.restoreDynamics()
    
    return "Physics is resumed"
