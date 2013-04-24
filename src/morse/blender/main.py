import logging; logger = logging.getLogger("morse." + __name__)
from morse.helpers.morse_logging import SECTION, ENDSECTION
import sys
import os
import time
import imp

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
    persistantstorage.datastreamDict = {}

    # this dictionary stores, for each components, the direction and the
    # configured datastream interfaces. Direction is 'IN' for streams
    # that are read by MORSE (typically, for actuators), and 'OUT'
    # for streams published by MORSE (typically, for sensors)
    persistantstorage.datastreams = {}

    # Create a dictionnary with the overlaid used
    persistantstorage.overlayDict = {}

    # Create the 'request managers' manager
    persistantstorage.morse_services = MorseServices()


    scene = morse.core.blenderapi.scene()

    # Store the position and orientation of all objects
    for obj in scene.objects:
        if obj.parent == None:
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

    if not (persistantstorage.robotDict or \
            persistantstorage.externalRobotDict): # No robot!
        logger.error("INITIALIZATION ERROR: no robot in your simulation!"
                     "Do not forget that components _must_ belong to a"
                     "robot (you can not have free objects)")
        return False

    
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

    if persistantstorage.datastreamDict:
        logger.info ("Datastream interfaces configured:")
        for obj, datastream_variables in persistantstorage.datastreamDict.items():
            logger.info ("\t- '{0}'".format(obj))

    logger.info("------------------------------------")
    logger.info ("")


def get_components_of_type(classname):
    components = []
    for component in persistantstorage.componentDict.values():
        logger.debug("Get component for class " + component.name() + ": " + component.__class__.__name__)
        if component.__class__.__name__ == classname:
            components.append(component)
    
    return components


def get_datastream_of_type(classname):
    for datastream_instance in persistantstorage.datastreamDict.values():
        if datastream_instance.__class__.__name__ == classname:
            return datastream_instance
    return None
    

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

        # If the list contains only strings, insert the list inside another one.
        # This is done for backwards compatibility with the previous
        #  syntax that allowed only one middleware per component
        if isinstance (datastream_list[0], str):
            datastream_list = [datastream_list]

        # What is the direction of our stream?
        # -> for Sensors, they *publish*,
        # -> for Actuator, they *read*
        if isinstance(instance, Sensor):
            direction = OUT
        elif isinstance(instance, Actuator):
            direction = IN
        else:
            assert(False)

        persistantstorage.datastreams[component_name] = (direction, 
                                     [d[0] for d in datastream_list])

        # Register all datastream's in the list
        for datastream_data in datastream_list:

            datastream_name = datastream_data[0]
            logger.info("Component: '%s' using datastream '%s'" % (component_name, datastream_name))
            found = False
            missing_component = False
            
            # Look for the listed datastream in the dictionary of active datastream's
            for datastream_obj, datastream_instance in persistantstorage.datastreamDict.items():
                logger.debug("Looking for '%s' in '%s'" % (datastream_name, datastream_obj))
                if datastream_name in datastream_obj:
                    found = True
                    # Make the datastream object take note of the component
                    break

            if not found:
                datastream_instance = create_instance(datastream_name)
                if datastream_instance != None:
                    persistantstorage.datastreamDict[datastream_name] = datastream_instance
                    logger.info("\tDatastream interface '%s' created" % datastream_name)
                else:
                    logger.error("INITIALIZATION ERROR: Datastream '%s' module"
                                 " could not be found! \n"
                                 " Could not import modules required for the "
                                 "desired datastream interface. Check that "
                                 "they can be found inside your PYTHONPATH "
                                 "variable.")
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
            logger.info("Component: '%s' operated by '%s'" %
                        (component_name, modifier_name))
            # Make the modifier object take note of the component
            modifier_instance = register_modifier(modifier_name, instance,
                                                  mod_data[1])
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
        node_name = os.uname()[1]

    logger.info ("This is node '%s'" % node_name)
    # Create the instance of the node class

    persistantstorage.node_instance = create_instance(classpath, \
            node_name, server_address, server_port)

def init(contr):
    """ General initialization of MORSE

    Here, all components, modifiers and middlewares are initialized.
    """
    
    init_logging()

    logger.log(SECTION, 'PRE-INITIALIZATION')
    # Get the version of Python used
    # This is used to determine also the version of Blender
    persistantstorage.pythonVersion = sys.version_info
    logger.info ("Python Version: %s.%s.%s" %
                    persistantstorage.pythonVersion[:3])
    logger.info ("Blender Version: %s.%s.%s" % morse.core.blenderapi.version())
    logger.info  ("Python path: %s" % sys.path)
    logger.info ("PID: %d" % os.getpid())

    persistantstorage.morse_initialised = False
    persistantstorage.base_clock = time.clock()
    persistantstorage.current_time = 0.0
    # Variable to keep trac of the camera being used
    persistantstorage.current_camera_index = 0

    init_ok = True
    init_ok = init_ok and create_dictionaries()

    logger.log(SECTION, 'SUPERVISION SERVICES INITIALIZATION')
    init_ok = init_ok and init_supervision_services()


    logger.log(SECTION, 'SCENE INITIALIZATION')
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

    if MULTINODE_SUPPORT:
        init_multinode()
    
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

    ###
    # First, load and map the socket request manager to the pseudo
    # 'simulation' component:
    try:
        request_manager = "morse.middleware.socket_request_manager.SocketRequestManager"
        if not persistantstorage.morse_services.add_request_manager(request_manager):
            return False
        # The simulation mangement services always uses at least sockets for requests.
        persistantstorage.morse_services.register_request_manager_mapping(
                "simulation", request_manager)
        persistantstorage.morse_services.register_request_manager_mapping(
                "communication", request_manager)

    except MorseServiceError as e:
        #...no request manager :-(
        logger.critical(str(e))
        logger.critical("SUPERVISION SERVICES INITIALIZATION FAILED")
        return False

    ###
    # Then, load any other middleware request manager that was declared
    # to also handle the 'simulation' pseudo-component:
    try:
        request_managers = component_config.component_service["simulation"]

        for request_manager in request_managers:
            try:
                # Load required request managers
                if not persistantstorage.morse_services.add_request_manager(request_manager):
                    return False

                persistantstorage.morse_services.register_request_manager_mapping("simulation", request_manager)
                logger.info("Adding '%s' to the middlewares for simulation "
                            "control" % request_manager)
            except MorseServiceError as e:
                #...no request manager :-(
                logger.critical(str(e))
                logger.critical("SUPERVISION SERVICES INITIALIZATION FAILED")
                return False

    except (AttributeError, NameError, KeyError): 
        # Nothing to declare: skip to the next step.
        pass


    ###
    # Services can be imported *only* after persistantstorage.morse_services
    # has been created. Else @service won't know where to register the RPC
    # callbacks.
    import morse.services.supervision_services
    import morse.services.communication_services

    logger.log(ENDSECTION, "SUPERVISION SERVICES INITIALIZED")
    return True


def simulation_main(contr):
    """ This method is called at every simulation step.

    We do here all homeworks to manage the simulation at whole.
    """
    # Update the time variable
    try:
        persistantstorage.current_time = time.clock() - \
                                         persistantstorage.base_clock
    except AttributeError:
        # If the 'base_clock' variable is not defined, there probably was
        #  a problem while doing the init, so we'll abort the simulation.
        logger.critical("INITIALIZATION ERROR: failure during initialization "
                        "of the simulator. Check the terminal for error "
                        "messages, and report them on the morse-dev@laas.fr "
                        "mailing list.")
        quit(contr)

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
        index = persistantstorage.current_camera_index
        next_camera = scene.cameras[index]
        scene.active_camera = next_camera
        logger.info("Showing view from camera: '%s'" % next_camera.name)
        # Disable mouse cursor for Human camera
        if next_camera.name == "Human_Camera":
            morse.core.blenderapi.mousepointer(visible = False)
        else:
            morse.core.blenderapi.mousepointer(visible = True)
        # Update the index for the next call
        index = (index + 1) % len(scene.cameras)
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

    logger.log(ENDSECTION, 'CLOSING DATASTREAMS...')
    # Force the deletion of the datastream objects
    if 'datastreamDict' in persistantstorage:
        for obj, datastream_instance in persistantstorage.datastreamDict.items():
            if datastream_instance:
                import gc # Garbage Collector
                logger.debug("At closing time, %s has %s references" %
                        (datastream_instance,
                         gc.get_referents(datastream_instance)))
                del obj

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
        b_obj.suspendDynamics()
        b_obj.setLinearVelocity([0.0, 0.0, 0.0], True)
        b_obj.setAngularVelocity([0.0, 0.0, 0.0], True)
        b_obj.applyForce([0.0, 0.0, 0.0], True)
        b_obj.applyTorque([0.0, 0.0, 0.0], True)

        logger.debug("%s goes to %s" % (b_obj, state[0]))
        b_obj.worldPosition = state[0]
        b_obj.worldOrientation = state[1]
        # Reset physics simulation
        b_obj.restoreDynamics()
