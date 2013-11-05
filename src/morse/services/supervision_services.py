import logging; logger = logging.getLogger("morse." + __name__)
from morse.core.services import service
from morse.core import status, blenderapi
from morse.blender.main import reset_objects as main_reset, close_all as main_close, quit as main_terminate
from morse.core.abstractobject import AbstractObject
from morse.core.exceptions import *
import json

class Supervision(AbstractObject):
    def __init__(self):
        super(Supervision, self).__init__()

    def name(self):
        return "simulation"

    @service
    def list_robots(self):
        """ Return a list of the robots in the current scenario

        Uses the list generated during the initialisation of the scenario
        """
        return [obj.name for obj in blenderapi.persistantstorage().robotDict.keys()]

    @service
    def reset_objects(self):
        """ Restore all simulation objects to their original position

        Upon receiving the request using sockets, call the
        'reset_objects' function located in morse/blender/main.py
        """
        contr = blenderapi.controller()
        main_reset(contr)
        return "Objects restored to initial position"

    @service
    def quit(self):
        """ Cleanly quit the simulation
        """
        contr = blenderapi.controller()
        main_close(contr)
        main_terminate(contr)

    @service
    def terminate(self):
        """ Terminate the simulation (no finalization done!)
        """
        contr = blenderapi.controller()
        main_terminate(contr)

    @service
    def activate(self, component_name):
        """ Enable the functionality of the component specified
        """
        try:
            blenderapi.persistantstorage().componentDict[component_name]._active = True
        except KeyError as detail:
            logger.warn("Component %s not found. Can't activate" % detail)
            raise MorseRPCTypeError("Component %s not found. Can't activate" % detail)

    @service
    def deactivate(self, component_name):
        """ Stop the specified component from calling its default_action method
        """
        try:
            blenderapi.persistantstorage().componentDict[component_name]._active = False
        except KeyError as detail:
            logger.warn("Component %s not found. Can't deactivate" % detail)
            raise MorseRPCTypeError("Component %s not found. Can't deactivate" % detail)

    @service
    def suspend_dynamics(self):
        """ Suspends physics for all object in the scene.
        """

        scene = blenderapi.scene()
        for object in scene.objects:
            object.suspendDynamics()

        return "Physics is suspended"

    @service
    def restore_dynamics(self):
        """ Resumes physics for all object in the scene.
        """

        scene = blenderapi.scene()
        for object in scene.objects:
            object.restoreDynamics()

        return "Physics is resumed"

    @service
    def details(self):
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


    @service
    def set_log_level(self, component, level):
        """
        Allow to change the logger level of a specific component

        :param string component: the name of the logger you want to modify
        :param string level: the desired level of logging
        """

        my_logger = logging.getLogger('morse.' + component)
        try:
            my_logger.setLevel(level)
        except ValueError as exn:
            raise MorseRPCInvokationError(str(exn))


    def get_structured_children_of(blender_object):
        """ Returns a nested dictionary of the given objects children, recursively.
        The retun format is as follows:

        {blender_object.name: [children_dictionary, position, orientation]}

        where children_dictionary is another of the same format, but with the keys
        being the children of blender_object. This continues down the entire tree
        structure.

        :param KX_GameObject blender_object: The Blender object to return children
        for.
        """
        children = blender_object.children
        orientation = blender_object.worldOrientation.to_quaternion()
        position = blender_object.worldPosition
        structure = { blender_object.name: [{},
                                            (position.x, position.y, position.z),
                                            (orientation.x, orientation.y,
                                             orientation.z, orientation.w)
                                            ]
                    }
        for c in children:
            structure[blender_object.name][0].update(
                get_structured_children_of(c) )
        return structure

    @service
    def get_scene_objects(self):
        """ Returns a hierarchial dictonary structure of all objects in the scene
        along with their positions and orientations, formated as a Python string
        representation.
        The structure:
        {object_name: [dict_of_children, position_tuple, quaternion_tuple],
        object_name: [dict_of_children, position_tuple, quaternion_tuple],
        ...}
        """

        scene = blenderapi.scene()
        # Special Morse items to remove from the list
        remove_items = ['Scene_Script_Holder', 'CameraFP', '__default__cam__']
        top_levelers = [o for o in scene.objects
                        if o.parent is None and
                        not o.name in remove_items]

        objects = {}
        for obj in top_levelers:
            objects.update(get_structured_children_of(obj))

        return objects

    def get_obj_by_name(name):
        """
        Return object in the scene associated to :param name:
        If it does not exists, throw a MorseRPCInvokationError
        """
        scene = blenderapi.scene()
        if name not in scene.objects:
            raise MorseRPCInvokationError(
                    "Object '%s' does not appear in the scene." % name)
        return scene.objects[name]

    @service
    def set_object_visibility(self, object_name, visible, do_children):
        """ Set the visibility of an object in the simulation.

        Note: The object will still have physics and dynamics despite being invisible.

        :param string object_name: The name of the object to change visibility of.
        :param visible boolean: Make the object visible(True) or invisible(False)
        :param do_children boolean: If True then the visibility of all children of
        object_name is also set."""

        blender_object = get_obj_by_name(object_name)
        blender_object.setVisible(visible, do_children)
        return visible

    @service
    def set_object_dynamics(self, object_name, state):
        """ Enable or disable the dynamics for an individual object.

        Note: When turning on dynamics, the object will continue with the velocities
        it had when it was turned off.

        :param string object_name: The name of the object to change.
        :param state boolean: Turn on dynamics(True), or off (False)
        """

        blender_object = get_obj_by_name(object_name)
        if state:
            blender_object.restoreDynamics()
        else:
            blender_object.suspendDynamics()
        return state

    def action(self):
        pass
