import logging; logger = logging.getLogger("morse." + __name__)
from morse.helpers.morse_logging import SECTION, ENDSECTION
import sys
import os
import imp
from subprocess import Popen, PIPE

# Force the full import of blenderapi so python computes correctly all
# values in its  namespace
import morse.core.blenderapi

# force a reload, since 'blenderapi' may have been already loaded
# *outside* the GameEngine
imp.reload(morse.core.blenderapi)
persistantstorage = morse.core.blenderapi.persistantstorage()

# The service management
from morse.core.services import MorseServices
from morse.core.sensor import Sensor
from morse.core.actuator import Actuator
from morse.core.modifier import register_modifier
from morse.helpers.loading import create_instance, create_instance_level
from morse.core.morse_time import TimeStrategies
from morse.core.zone import ZoneManager

# Override the default Python exception handler
def morse_excepthook(*args, **kwargs):
    logger.error("[ERROR][MORSE] Uncaught exception, quit Blender.", exc_info = tuple(args))
    # call default python exception hook
    # on Ubuntu/Python3.4 sys.excepthook is overriden by `apport_excepthook`
    sys.__excepthook__(*args, **kwargs)
    import os
    os._exit(-1)

# Uncaught exception quit BGE
sys.excepthook = morse_excepthook

# Constants for stream directions
IN = 'IN'
OUT = 'OUT'

# The file component_config.py is at the moment included
#  in the .blend file of the scene
try:
    import component_config
except ImportError as detail:
    logger.warning("%s.\nNo datastream/services/modifiers will be configured."
                    "\nMake sure the script 'component_config.py' is present"
                    "in the .blend file." % detail)

MULTINODE_SUPPORT = False
# The file multinode_config.py is at the moment included
#  in the .blend file of the scene
# Used to setup the multinode information
try:
    import multinode_config
    MULTINODE_SUPPORT = True
except ImportError as detail:
    logger.info("No multi-node scene configuration file found. "
                "Multi-node support disabled.")

from morse.core.exceptions import MorseServiceError

def no_op():
    pass

def _associate_child_to_robot(obj, robot_instance, unset_default):
    """ Reference the link of all obj to their associated robot_instance.
        If it is an external robot_instance, unset default_action
    """
    # Create an empty list for the components of this robot
    robot_instance.components = []
    for child in obj.childrenRecursive:
        try:
            # Look for the components tagged as such
            child['Component_Tag']
        except KeyError:
            continue

        robot_instance.components.append(child)

        if not 'classpath' in child:
            logger.error("No 'classpath' in child %s\n  Please make sure you "
                         "are using the new builder classes"%str(child.name))
            return False
        # Create an instance of the component class
        #  and add it to the component list of persistantstorage()
        instance = create_instance_level(child['classpath'],
                                         child.get('abstraction_level'),
                                         child, robot_instance)
        if instance:
            persistantstorage.componentDict[child.name] = instance
        else:
            logger.error("INITIALIZATION ERROR: the component '%s'"
                         " could not be properly initialized. Error when "
                         "creating the class instance", obj.name)
            return False

        # Unset the default action of components of external robots
        if unset_default:
            instance.default_action = no_op
            logger.info("Component " + child.name + " disabled: parent "  \
                                     + obj.name + " is an External robot.")
        else:
            logger.info("Component %s %s added to %s" %
                        (child.name,
                         "(level: %s)" % child.get("abstraction_level") \
                                 if child.get("abstraction_level") else "",
                         obj.name)
                       )

    return True

# Create a list of the robots in the scene
def create_dictionaries ():
    """Creation of a list of all the robots and components in the scene.
       Uses the properties of the objects to determine what they are."""

    # Create a dictionary that stores initial positions of all objects
    # in the simulation, used to reset the simulation.
    persistantstorage.blender_objects = {}

    # Create a dictionary of the components in the scene
    persistantstorage.componentDict = {}

    # Create a dictionary of the robots in the scene
    persistantstorage.robotDict = {}

    # Create a dictionary of the external robots in the scene
    # Used for the multi-node simulation
    persistantstorage.externalRobotDict = {}

    # Create a dictionnary with the passive, but interactive (ie, with an
    # 'Object' property) objects in the scene.
    persistantstorage.passiveObjectsDict = {}

    # Create a dictionary with the modifiers
    persistantstorage.modifierDict = {}

    # Create a dictionary with the datastream interfaces used
    persistantstorage.stream_managers = {}

    # this dictionary stores, for each components, the direction and the
    # configured datastream interfaces. Direction is 'IN' for streams
    # that are read by MORSE (typically, for actuators), and 'OUT'
    # for streams published by MORSE (typically, for sensors)
    persistantstorage.datastreams = {}

    # Create a dictionnary with the overlaid used
    persistantstorage.overlayDict = {}

    # Create a dictionnary for the 'service object', such as supervision
    persistantstorage.serviceObjectDict = {}

    # Create the 'request managers' manager
    persistantstorage.morse_services = MorseServices()

    # Create the zone manager
    persistantstorage.zone_manager = ZoneManager()

    scene = morse.core.blenderapi.scene()

    # Store the position and orientation of all objects
    for obj in scene.objects:
        if obj.parent is None:
            import mathutils
            pos = mathutils.Vector(obj.worldPosition)
            ori = mathutils.Matrix(obj.worldOrientation)
            persistantstorage.blender_objects[obj] = [pos, ori]

    # Get the list of passive interactive objects.

    # These objects have a 'Object' property set to true
    # (plus several other optional properties).
    # See the documentation for the up-to-date list
    # (doc/morse/user/others/passive_objects.rst) -- or read the code below :-)
    for obj in scene.objects:
        # Check the object has an 'Object' property set to true
        if 'Object' in obj and obj['Object']:
            details = {
                       'label': obj['Label'] if 'Label' in obj else str(obj),
                       'description': obj['Description'] if 'Description' in obj else "",
                       'type': obj['Type'] if 'Type' in obj else "Object",
                       'graspable': obj['Graspable'] if 'Graspable' in obj else False
                      }
            persistantstorage.passiveObjectsDict[obj] = details
            logger.info("Added {name} as a {graspable}active object".format(
                                 name = details['label'],
                                 graspable = "graspable " if details['graspable'] else ""))

    if not persistantstorage.passiveObjectsDict:
        logger.info("No passive objects in the scene.")

    # Get the robots
    for obj in scene.objects:
        if 'Robot_Tag' in obj or 'External_Robot_Tag' in obj:
            if not 'classpath' in obj:
                logger.error("No 'classpath' in %s\n  Please make sure you are "
                             "using the new builder classes"%str(obj.name))
                return False
            # Create an object instance and store it
            instance = create_instance_level(obj['classpath'],
                                             obj.get('abstraction_level'),
                                             obj)

            if not instance:
                logger.error("Could not create %s"%str(obj['classpath']))
                return False
            # store instance in persistant storage dictionary
            if 'Robot_Tag' in obj:
                persistantstorage.robotDict[obj] = instance
            else:
                persistantstorage.externalRobotDict[obj] = instance

    if not (persistantstorage.robotDict or
            persistantstorage.externalRobotDict): # No robot!
        logger.error("INITIALIZATION ERROR: no robot in your simulation!"
                     "Do not forget that components _must_ belong to a"
                     "robot (you can not have free objects)")
        return False

    # Get the zones
    for obj in scene.objects:
        if 'Zone_Tag' in obj:
            persistantstorage.zone_manager.add(obj)

    # Get the robot and its instance
    for obj, robot_instance in persistantstorage.robotDict.items():
        if not _associate_child_to_robot(obj, robot_instance, False):
            return False

    # Get the external robot and its instance
    for obj, robot_instance in persistantstorage.externalRobotDict.items():
        if not _associate_child_to_robot(obj, robot_instance, True):
            return False

    # Check we have no 'free' component (they all must belong to a robot)
    for obj in scene.objects:
        try:
            obj['Component_Tag']
            if obj.name not in persistantstorage.componentDict.keys():
                logger.error("INITIALIZATION ERROR: the component '%s' "
                             "does not belong to any robot: you need to fix "
                             "that by parenting it to a robot." % obj.name)
                return False
        except KeyError as detail:
            pass

    # Will return true always (for the moment)
    return True


def check_dictionaries():
    """ Print the contents of the robot and component dictionaries."""
    logger.info ("")
    logger.info("------------------------------------")
    logger.info("-        SIMULATION SUMMARY        -")
    logger.info("------------------------------------")
    logger.info("Robots in the simulation:")
    for obj, robot_instance in persistantstorage.robotDict.items():
        logger.info("\tROBOT: '{0}'".format(obj))
        for component in robot_instance.components:
            logger.info ("\t\t- Component: '{0}'".format(component))

    if MULTINODE_SUPPORT:
        logger.info ("External robots (from other simulation nodes):")
        for obj, robot_position in persistantstorage.externalRobotDict.items():
            logger.info ("\tROBOT: '{0}'".format(obj))

    logger.info ("Available services:")

    if persistantstorage.morse_services.request_managers():
        for name, instance in persistantstorage.morse_services.request_managers().items():
            logger.info ("\t- Interface {0}".format(name))
            for component, service in instance.services().items():
                logger.info ("\t\t- %s: %s" % (component,service))

    else:
        logger.info ("\tNone")

    logger.info ("Modifiers in use:")
    if persistantstorage.modifierDict:
        for obj, modifier_variables in persistantstorage.modifierDict.items():
            logger.info ("\t- '{0}'".format(obj))
    else:
        logger.info ("\tNone")


    logger.info ("")

    if persistantstorage.stream_managers:
        logger.info ("Datastream interfaces configured:")
        for key in persistantstorage.stream_managers.keys():
            logger.info ("\t- '%s'" % key)

    logger.info("------------------------------------")
    logger.info ("")

def load_datastream_manager(datastream_name):
    datastream_instance = persistantstorage.stream_managers.get(datastream_name, None)
    if not datastream_instance:
        kwargs = component_config.stream_manager.get(datastream_name, {})
        try:
            datastream_instance = create_instance(datastream_name, None, kwargs)
        except Exception as e:
            logger.error("Catched exception %s in the construction of %s" %
                         (e, datastream_name))
            return None 

        if datastream_instance:
            persistantstorage.stream_managers[datastream_name] = datastream_instance
            logger.info("\tDatastream interface '%s' created" % datastream_name)
        else:
            logger.error("INITIALIZATION ERROR: Datastream '%s' module"
                         " could not be found! \n"
                         " Could not import modules required for the "
                         "desired datastream interface. Check that "
                         "they can be found inside your PYTHONPATH "
                         "variable." % datastream_name)
            return None
    return datastream_instance

def link_datastreams():
    """ Read the configuration script (inside the .blend file)
        and assign the correct datastream and options to each component. """
    try:
        component_list = component_config.component_datastream
    except (AttributeError, NameError) as detail:
        # Exit gracefully if there are no datastream specified
        logger.info ("No datastream section found in configuration file.")
        return True

    #for component_name, datastream_data in component_list.items():
    for component_name, datastream_list in component_list.items():
        # Get the instance of the object
        try:
            instance = persistantstorage.componentDict[component_name]
        except KeyError as detail:
            logger.error ("Component listed in component_config.py not found "
                          "in scene: %s" % detail)
            logger.error("INITIALIZATION ERROR: your configuration file is "
                         " not valid. Please check the name of your components "
                         " and restart the simulation.")
            return False

        # Do not configure middlewares for components that are external,
        #  that is, they are handled by another node.
        try:
            instance.robot_parent.bge_object['Robot_Tag']
            # If the robot is external, it will have the 'External_Robot_Tag'
            #  instead, and this test will fail
        except KeyError as detail:
            # Skip the configuration of this component
            continue

        persistantstorage.datastreams[component_name] = datastream_list

        # Register all datastream's in the list
        for datastream_data in datastream_list:

            datastream_name = datastream_data[0]
            logger.info("Component: '%s' using datastream '%s'" % (component_name, datastream_name))

            datastream_instance = load_datastream_manager(datastream_name)
            if not datastream_name:
                return False

            datastream_instance.register_component(component_name, instance, datastream_data)

    # Will return true always (for the moment)
    return True


def link_services():
    """ Read the configuration script (inside the .blend file)
        and assign the correct service handlers and options to each component.
    """
    try:
        component_list = component_config.component_service
    except (AttributeError, NameError) as detail:
        # Exit gracefully if there are no services specified
        logger.info("No service section found in configuration file.")
        return True

    for component_name, request_manager_data in component_list.items():
        # Get the instance of the object

        if component_name == "simulation": # Special case for the pseudo-component 'simulation'
            continue

        try:
            instance = persistantstorage.componentDict[component_name]
        except KeyError as detail:
            try:
                scene = morse.core.blenderapi.scene()
                robot_obj = scene.objects[component_name]
                instance = persistantstorage.robotDict[robot_obj]

            except KeyError as detail:
                logger.error("Component listed in component_config.py "
                             "not found in scene: %s" % detail)
                logger.error("INITIALIZATION ERROR: the component_services "
                             "section of your configuration file is not valid."
                             "Please check the name of your components and "
                             "restart the simulation.")
                return False

        for request_manager in request_manager_data:
            # Load required request managers
            if not persistantstorage.morse_services.add_request_manager(request_manager):
                return False

            persistantstorage.morse_services.register_request_manager_mapping(component_name, request_manager)
            instance.register_services()
            logger.info("Component: '%s' using middleware '%s' for services" %
                        (component_name, request_manager))

    return True


def load_overlays():
    """ Read and initialize overlays from the configuration script.
    """

    try:
        overlays_list = component_config.overlays
    except (AttributeError, NameError) as detail:
        # Exit gracefully if there are no services specified
        logger.info("No overlay section found in configuration file.")
        return True

    for request_manager_name, overlays in overlays_list.items():
        for overlaid_name, overlay_details in overlays.items():
            overlay_name, kwargs = overlay_details

            try:
                overlaid_object = persistantstorage.componentDict[overlaid_name]
            except KeyError:
                logger.error("Could not find the object to overlay: %s." %
                              overlaid_name)
                return False

            # Instanciate the overlay, passing the overlaid object to
            # the constructor + any optional arguments
            instance = create_instance(overlay_name, overlaid_object, **kwargs)
            persistantstorage.morse_services.register_request_manager_mapping(
                    instance.name(), request_manager_name)
            instance.register_services()
            persistantstorage.overlayDict[overlay_name] = instance
            logger.info("Component '%s' overlaid with '%s' using middleware "
                        "'%s' for services" %
                        (overlaid_object.name(),
                         overlay_name,
                         request_manager_name))
    return True


def add_modifiers():
    """ Read the configuration script (inside the .blend file)
        and assign the correct data modifiers to each component. """
    try:
        component_list = component_config.component_modifier
    except (AttributeError, NameError) as detail:
        # Exit gracefully if there are no modifiers specified
        logger.info("No modifiers section found in configuration file")
        return True

    for component_name, mod_list in component_list.items():
        # Get the instance of the object
        try:
            instance = persistantstorage.componentDict[component_name]
        except KeyError as detail:
            logger.warning("Component listed in component_config.py not "
                           "found in scene: %s" % detail)
            continue

        for mod_data in mod_list:
            modifier_name = mod_data[0]
            direction = mod_data[1]
            logger.info("Component: '%s' operated by '%s'" %
                        (component_name, modifier_name))
            # Make the modifier object take note of the component
            modifier_instance = register_modifier(modifier_name, instance,
                                                  direction, mod_data[2])
            if not modifier_instance:
                return False
            persistantstorage.modifierDict[modifier_name] = modifier_instance

    return True

def init_multinode():
    """
    Initializes the MORSE node in a Multinode configuration.
    """
    logger.log(SECTION, 'MULTINODE INITIALIZATION')
    # Configuration for the multi-node simulation
    try:
        protocol = multinode_config.node_config["protocol"]
    except (NameError, AttributeError) as detail:
        protocol = "socket"

    # Get the correct class reference according to the chosen protocol
    if protocol == "socket":
        classpath = "morse.multinode.socket.SocketNode"
    elif protocol == "hla":
        classpath = "morse.multinode.hla.HLANode"

    try:
        server_address = multinode_config.node_config["server_address"]
        server_port = int(multinode_config.node_config["server_port"])
    except (NameError, AttributeError) as detail:
        logger.warning("No node configuration found. Using default values for "
                       "this simulation node.\n\tException: ", detail)
        server_address = "localhost"
        server_port = 65000

    try:
        node_name = multinode_config.node_config["node_name"]
    except (NameError, AttributeError) as detail:
        logger.warning("No node name defined. Using host name.\n"
                        "\tException: ", detail)
        import socket
        node_name = socket.gethostname()

    logger.info ("This is node '%s'" % node_name)
    # Create the instance of the node class

    persistantstorage.node_instance = create_instance(classpath,
                                                      node_name, server_address, server_port)

class MorseSyncProcess:
    def __init__(self):
        args = ['morse_sync', '-p', str(1.0/morse.core.blenderapi.getfrequency())]
        socket_manager = 'morse.middleware.socket_datastream.SocketDatastreamManager'
        socket_properties = component_config.stream_manager[socket_manager]
        if 'sync_port' in socket_properties:
            args = args + ['-P', str(socket_properties['sync_port'])]
        self.proc = Popen(args, stdin=PIPE)
        # always load datastream manager if we use use_internal_syncer
        load_datastream_manager(socket_manager)

    def set_period(self, new_value):
        msg = "set_period " + str(new_value) + "\n\n"
        self.proc.stdin.write(bytes(msg, encoding='ascii'))
        self.proc.stdin.flush()

    def __del__(self):
        self.proc.communicate(b"quit", timeout = None)

def init(contr):
    """ General initialization of MORSE

    Here, all components, modifiers and middlewares are initialized.
    """

    init_logging()

    logger.log(SECTION, 'PRE-INITIALIZATION')
    # Get the version of Python used
    # This is used to determine also the version of Blender
    persistantstorage.pythonVersion = sys.version_info
    logger.info("Python Version: %s.%s.%s" % persistantstorage.pythonVersion[:3])
    logger.info("Blender Version: %s.%s.%s" % morse.core.blenderapi.version())
    logger.info("Python path: %s" % sys.path)
    logger.info("PID: %d" % os.getpid())

    persistantstorage.morse_initialised = False
    persistantstorage.time = TimeStrategies.make(morse.core.blenderapi.getssr()['time_management'],
                                                 morse.core.blenderapi.getssr().get('use_relative_time', False))
    # Variable to keep trac of the camera being used
    persistantstorage.current_camera_index = 0

    init_ok = True
    init_ok = init_ok and create_dictionaries()

    persistantstorage.internal_syncer = None

    logger.log(SECTION, 'SUPERVISION SERVICES INITIALIZATION')
    init_ok = init_ok and init_supervision_services()

    # Make sure to start the internal syncer after initialization of
    # time_scale (done in in init_supervision_services)
    try:
        use_ = morse.core.blenderapi.getssr()['use_internal_syncer']
        if use_:
            persistantstorage.internal_syncer = MorseSyncProcess()
    except KeyError:
        pass

    logger.log(SECTION, 'SCENE INITIALIZATION')

    if MULTINODE_SUPPORT:
        init_multinode()

    init_ok = init_ok and link_services()
    init_ok = init_ok and add_modifiers()
    init_ok = init_ok and link_datastreams()
    init_ok = init_ok and load_overlays()

    if init_ok:
        check_dictionaries()
        persistantstorage.morse_initialised = True
        logger.log(ENDSECTION, 'SCENE INITIALIZED')
    else:
        logger.critical('INITIALIZATION FAILED!')
        logger.info("Exiting now.")
        contr = morse.core.blenderapi.controller()
        close_all(contr)
        quit(contr)

    # Set the default value of the logic tic rate to 60
    #bge.logic.setLogicTicRate(60.0)
    #bge.logic.setPhysicsTicRate(60.0)

def init_logging():
    from morse.core.ansistrm import ColorizingStreamHandler

    if "with-colors" in sys.argv:
        if "with-xmas-colors" in sys.argv:
            ch = ColorizingStreamHandler(scheme = "xmas")
        elif "with-reverse-colors" in sys.argv:
            ch = ColorizingStreamHandler(scheme = "dark")
        else:
            ch = ColorizingStreamHandler()

    else:
        ch = ColorizingStreamHandler(scheme = "mono")

    from morse.helpers.morse_logging import MorseFormatter
    # create logger
    logger = logging.getLogger('morse')
    logger.setLevel(logging.INFO)

    # create console handler and set level to debug

    ch.setLevel(logging.DEBUG)

    # create formatter
    formatter = MorseFormatter('%(message)s')

    # add formatter to ch
    ch.setFormatter(formatter)

    # add ch to logger
    logger.addHandler(ch)

def init_supervision_services():
    """ This method first loads the socket service manager, map the
    virtual 'simulation' component to it, loads any other request
    manager mapped to the 'simulation' component and register all
    simulation management services declared in
    :py:mod:`morse.core.supervision_services`
    """

    from morse.services.supervision_services import Supervision
    from morse.services.communication_services import Communication
    from morse.services.time_services import TimeServices

    simulation_service = Supervision()
    communication_service = Communication()
    time_service= TimeServices()

    try:
        time_scale = morse.core.blenderapi.getssr()['time_scale']
        time_service.set_time_scale(time_scale)
    except KeyError as e:
        pass

    persistantstorage.serviceObjectDict[simulation_service.name()] = simulation_service
    persistantstorage.serviceObjectDict[communication_service.name()] = communication_service
    persistantstorage.serviceObjectDict[time_service.name()] = time_service

    # For each entries of serviceObjects, register the service as
    # requested by configuration + socket middleware i/o.
    try:
        for key, services in persistantstorage.serviceObjectDict.items():
            request_managers = component_config.component_service.get(key, [])
            request_managers.append("morse.middleware.socket_request_manager.SocketRequestManager")

            for request_manager in request_managers:
                try:
                    # Load required request managers
                    if not persistantstorage.morse_services.add_request_manager(request_manager):
                        return False

                    persistantstorage.morse_services.register_request_manager_mapping(key, request_manager)
                    logger.info("Adding '%s' to the middlewares for %s "
                                "control" % (request_manager, key))
                except MorseServiceError as e:
                    #...no request manager :-(
                    logger.critical(str(e))
                    logger.critical("SUPERVISION SERVICES INITIALIZATION FAILED")
                    return False

            services.register_services()

    except (AttributeError, NameError, KeyError):
        # Nothing to declare: skip to the next step.
        pass

    logger.log(ENDSECTION, "SUPERVISION SERVICES INITIALIZED")
    return True


def simulation_main(contr):
    """ This method is called at every simulation step.

    We do here all homeworks to manage the simulation at whole.
    """
    # Call datastream manager action handler
    # Call it early at the synchronisation management may be done here
    if 'stream_managers' in persistantstorage:
        for ob in persistantstorage.stream_managers.values():
            ob.action()

    # Update the time variable
    try:
        persistantstorage.time.update()
    except AttributeError:
        # If the 'base_clock' variable is not defined, there probably was
        #  a problem while doing the init, so we'll abort the simulation.
        logger.critical("INITIALIZATION ERROR: failure during initialization "
                        "of the simulator. Check the terminal for error "
                        "messages, and report them on the morse-dev@laas.fr "
                        "mailing list.")
        quit(contr)

    if 'serviceObjectDict' in persistantstorage:
        for ob in persistantstorage.serviceObjectDict.values():
            ob.action()

    if "morse_services" in persistantstorage:
        # let the service managers process their inputs/outputs
        persistantstorage.morse_services.process()

    if MULTINODE_SUPPORT:
        # Register the locations of all the robots handled by this node
        persistantstorage.node_instance.synchronize()


def switch_camera(contr):
    """ Cycle through the cameras in the scene during the game.
    """
    sensor = contr.sensors['F9_KEY']
    # Activate only once for each key press
    if sensor.positive and sensor.triggered:
        scene = morse.core.blenderapi.scene()
        cameras = [c for c in scene.cameras if not 'NOT_F9_ABLE' in c]
        index = persistantstorage.current_camera_index
        next_camera = cameras[index]
        scene.active_camera = next_camera
        logger.info("Showing view from camera: '%s'" % next_camera.name)
        # Update the index for the next call
        index = (index + 1) % len(cameras)
        persistantstorage.current_camera_index = index


def close_all(contr):
    """ Close the open communication channels from middlewares
    Call the destructors of all component instances. This should also call
    the methods to close middlewares
    """
    logger.log(ENDSECTION, 'COMPONENTS FINALIZATION')
    # Force the deletion of the sensor objects
    if 'componentDict' in persistantstorage:
        for component_instance in persistantstorage.componentDict.values():
            component_instance.finalize()

    # Force the deletion of the robot objects
    if 'robotDict' in persistantstorage:
        for robot_instance in persistantstorage.robotDict.values():
           robot_instance.finalize()

    logger.log(ENDSECTION, 'CLOSING REQUEST MANAGERS...')
    del persistantstorage.morse_services
    del persistantstorage.serviceObjectDict

    logger.log(ENDSECTION, 'CLOSING DATASTREAMS...')
    # Force the deletion of the datastream objects
    if 'stream_managers' in persistantstorage:
        for datastream_instance in persistantstorage.stream_managers.values():
            datastream_instance.finalize()
        del persistantstorage.stream_managers

    logger.log(ENDSECTION, 'CLOSING OVERLAYS...')
    del persistantstorage.overlayDict

    if MULTINODE_SUPPORT:
        logger.log(ENDSECTION, 'CLOSING MULTINODE...')
        persistantstorage.node_instance.finalize()


def finish(contr):
    """ Normal exit from the Game Engine, when pressing ESC key """
    sensor = contr.sensors['ESC_KEY']

    #execute only when the ESC key is released (if we don't test that,
    #the code get executed two time, when pressed, and when released)
    if not sensor.positive and sensor.triggered:
        close_all(contr)
        quit(contr)


def restart(contr):
    """ Call the Game Engine restart funcionality * DOES NOT WORK * """
    sensor = contr.sensors['F11_KEY']

    # Execute only when the F11 key is released (if we don't test that,
    #  the code get executed two times, when pressed, and when released)
    if not sensor.positive and sensor.triggered:

        logger.warning("Replacing everything at initial position")
        reset_objects(contr)
        return


def quit(contr):
    """ Exit graciously from the simulation """
    logger.log(ENDSECTION, 'EXITING SIMULATION')

    quitActuator = contr.actuators['Quit_sim']
    contr.activate(quitActuator)


def reset_objects(contr):
    """ Place all objects in the initial position

    Restore the position and rotation of objects and robots
    to their original state, during the simulation.
    """
    for b_obj, state in persistantstorage.blender_objects.items():
        # Stop physics simulation
        b_obj.setLinearVelocity([0.0, 0.0, 0.0], True)
        b_obj.setAngularVelocity([0.0, 0.0, 0.0], True)
        b_obj.applyForce([0.0, 0.0, 0.0], True)
        b_obj.applyTorque([0.0, 0.0, 0.0], True)

        b_obj.suspendDynamics()
        logger.debug("%s goes to %s" % (b_obj, state[0]))
        b_obj.worldPosition = state[0]
        b_obj.worldOrientation = state[1]
        # Reset physics simulation
        b_obj.restoreDynamics()

