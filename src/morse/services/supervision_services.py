import logging; logger = logging.getLogger("morse." + __name__)
from morse.core.services import service
from morse.core import status, blenderapi
from morse.blender.main import reset_objects as main_reset, close_all as main_close, quit as main_terminate
from morse.core.exceptions import *

@service(component = "simulation")
def list_robots():
    """ Return a list of the robots in the current scenario

    Uses the list generated during the initialisation of the scenario
    """
    return [obj.name for obj in blenderapi.persistantstorage().robotDict.keys()]

@service(component = "simulation")
def reset_objects():
    """ Restore all simulation objects to their original position

    Upon receiving the request using sockets, call the
    'reset_objects' function located in morse/blender/main.py
    """
    contr = blenderapi.controller()
    main_reset(contr)
    return "Objects restored to initial position"

@service(component = "simulation")
def quit():
    """ Cleanly quit the simulation
    """
    contr = blenderapi.controller()
    main_close(contr)
    main_terminate(contr)
    
@service(component = "simulation")
def terminate():
    """ Terminate the simulation (no finalization done!)
    """
    contr = blenderapi.controller()
    main_terminate(contr)

@service(component = "simulation")
def activate(component_name):
    """ Enable the functionality of the component specified
    """
    try:
        blenderapi.persistantstorage().componentDict[component_name]._active = True
    except KeyError as detail:
        logger.warn("Component %s not found. Can't activate" % detail)
        raise MorseRPCTypeError("Component %s not found. Can't activate" % detail)

@service(component = "simulation")
def deactivate(component_name):
    """ Stop the specified component from calling its default_action method
    """
    try:
        blenderapi.persistantstorage().componentDict[component_name]._active = False
    except KeyError as detail:
        logger.warn("Component %s not found. Can't deactivate" % detail)
        raise MorseRPCTypeError("Component %s not found. Can't deactivate" % detail)
        
@service(component = "simulation")
def suspend_dynamics():
    """ Suspends physics for all object in the scene.
    """

    scene = blenderapi.scene()
    for object in scene.objects:
        object.suspendDynamics()
        
    return "Physics is suspended"
        
@service(component = "simulation")
def restore_dynamics():
    """ Resumes physics for all object in the scene.
    """

    scene = blenderapi.scene()
    for object in scene.objects:
        object.restoreDynamics()
    
    return "Physics is resumed"

@service(component = "simulation")
def details():
    """Returns a structure containing all possible details
    about the simulation currently running, including
    the list of robots, the list of services and datastreams,
    the list of middleware in use, etc.
    """

    simu = blenderapi.persistantstorage()
    details = {}


    # Retrieves the list of services and associated middlewares
    services = {}
    services_iface = {}
    for n, i in simu.morse_services.request_managers().items():
        services.update(i.services())
        for cmpt in i.services():
            services_iface.setdefault(cmpt, []).append(n)

    def cmptdetails(c):
        c = simu.componentDict[c.name]
        cmpt = {"type": type(c).__name__,}
        if c.name() in services:
            cmpt["services"] = services[c.name()]
            cmpt["service_interfaces"] = services_iface[c.name()]

        if c.name() in simu.datastreams:
            stream = simu.datastreams[c.name()]
            cmpt["stream"] = stream[0]
            cmpt["stream_interfaces"] = stream[1]

        return cmpt

    def robotdetails(r):
        robot = {"name": r.name(),
                "type": type(r).__name__,
                "components": {c.name:cmptdetails(c) for c in r.components},
                }
        if r.name() in services:
            robot["services"] = services[r.name()]
            robot["services_interfaces"] = services_iface[r.name()]
        return robot
    
    for n, i in simu.datastreamDict.items():
        pass


    details['robots'] = [robotdetails(r) for n, r in simu.robotDict.items()]
    return details

