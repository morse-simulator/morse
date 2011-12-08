import logging; logger = logging.getLogger("morse." + __name__)
from morse.core.logging import SECTION, ENDSECTION
import sys
import os
import re
import time
import bpy
import GameLogic
# The service management
from morse.core.services import MorseServices

# The file component_config.py is at the moment included
#  in the .blend file of the scene
try:
    import component_config
    
except ImportError as detail:
    logger.warning("%s.\nNo middlewares/services/modifiers will be configured.\nMake sure the script 'component_config.py' is present in the .blend file." % detail)


MULTINODE_SUPPORT = False
# The file multinode_config.py is at the moment included
#  in the .blend file of the scene
# Used to setup the multinode information
try:
    import multinode_config
    MULTINODE_SUPPORT = True
except ImportError as detail:
    logger.info("No multi-node scene configuration file found. Multi-node support disabled.")

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

        robot_instance.components.append (child)

        # Create an instance of the component class
        #  and add it to the component list of GameLogic
        instance = create_instance (child, robot_instance)
        if instance != None:
            GameLogic.componentDict[child.name] = instance
        else:
            logger.error("""
    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    ERROR: the component '""" + obj.name + """' could not
    be properly initialized.
    There was an error when creating the class instance.
    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                """)
            return False

        # Unset the default action of components of external robots
        if unset_default:
            instance.default_action = no_op
            logger.info("Component " + child.name + " disabled: parent "  \
                                     + obj.name + " is an External robot.")

    return True

# Create a list of the robots in the scene
def create_dictionaries ():
    """Creation of a list of all the robots and components in the scene.
       Uses the properties of the objects to determine what they are."""

    # Create a dictionary that stores initial positions of all objects
    # in the simulation, used to reset the simulation.
    if not hasattr(GameLogic, 'blender_objects'):
        GameLogic.blender_objects = {}

    # Create a dictionary of the components in the scene
    if not hasattr(GameLogic, 'componentDict'):
        GameLogic.componentDict = {}

    # Create a dictionary of the robots in the scene
    if not hasattr(GameLogic, 'robotDict'):
        GameLogic.robotDict = {}

    # Create a dictionary of the external robots in the scene
    # Used for the multi-node simulation
    if not hasattr(GameLogic, 'externalRobotDict'):
        GameLogic.externalRobotDict = {}

    # Create a dictionnary with the passive, but interactive (ie, with an
    # 'Object' property) objects in the scene.
    if not hasattr(GameLogic, 'passiveObjectsDict'):
        GameLogic.passiveObjectsDict = {}

    # Create a dictionary with the modifiers
    if not hasattr(GameLogic, 'modifierDict'):
        GameLogic.modifierDict = {}

    # Create a dictionary with the middlewares used
    if not hasattr(GameLogic, 'mwDict'):
        GameLogic.mwDict = {}

    # Create a dictionary with the service used
    if not hasattr(GameLogic, 'serviceDict'):
        GameLogic.serviceDict = {}

    # Create a dictionnary with the overlaid used
    if not hasattr(GameLogic, 'overlayDict'):
        GameLogic.overlayDict = {}

    scene = GameLogic.getCurrentScene()

    # Store the position and orientation of all objects
    for obj in scene.objects:
        if obj.parent == None:
            import mathutils
            pos = mathutils.Vector(obj.worldPosition)
            ori = mathutils.Matrix(obj.worldOrientation)
            GameLogic.blender_objects[obj] = [pos, ori]

    # Get the list of passive interactive objects.

    # These objects have a 'Object' property set to true + several other optional
    # properties. See the documentation for the up-to-date list
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
            GameLogic.passiveObjectsDict[obj] = details
            logger.info("Added {name} as a {graspable}active object".format(
                                 name = details['label'],
                                 graspable = "graspable " if details['graspable'] else ""))

    if not GameLogic.passiveObjectsDict:
        logger.info("No passive objects in the scene.")

    # Get the robots
    for obj in scene.objects:
        try:
            obj['Robot_Tag']
            # Create an object instance and store it
            instance = create_instance (obj)
            if instance != None:
                GameLogic.robotDict[obj] = instance
            else:
                return False
        except KeyError as detail:
            pass
        try:
            obj['External_Robot_Tag']
            # Create an object instance and store it
            instance = create_instance (obj)
            if instance != None:
                GameLogic.externalRobotDict[obj] = instance
            else:
                return False
        except KeyError as detail:
            pass
    
    if not (GameLogic.robotDict or GameLogic.externalRobotDict): # No robot!
        logger.error("""
    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    ERROR: no robot in your simulation!
    
    Do not forget that components _must_ belong to a
    robot (you can not have free objects)
    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            """)
        return False

    
    # Get the robot and its instance
    for obj, robot_instance in GameLogic.robotDict.items():
        if not _associate_child_to_robot(obj, robot_instance, False):
            return False
    
    # Get the external robot and its instance
    for obj, robot_instance in GameLogic.externalRobotDict.items():
        if not _associate_child_to_robot(obj, robot_instance, True):
            return False
  
    # Check we have no 'free' component (they all must belong to a robot)
    for obj in scene.objects:
        try:
            obj['Component_Tag']
            if obj.name not in GameLogic.componentDict.keys():
                logger.error("""
    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    ERROR: the component '""" + obj.name + """' does not
    belong to any robot: you need to fix that by 
    parenting it to a robot.                    
    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                    """)
                return False
        except KeyError as detail:
            pass

    """ Modifiers are now dynamically created.
    The following lines should do nothing but are still there for backward
    compatibility.

    TODO: remove it when we are confident we can.
    """
    # Get the modifiers
    for obj in scene.objects:
        try:
            obj['Modifier_Tag']
            # Create an object instance and store it
            instance = create_instance (obj)
            if instance != None:
                GameLogic.modifierDict[obj] = instance
            else:
                return False
        except KeyError:
            pass
    
    """ Middlewares are now dynamically created.
    The following lines should do nothing but are still there for backward
    compatibility.

    TODO: remove it when we are confident we can.
    """
    # Get the middlewares
    for obj in scene.objects:
        is_middleware = False
        try:
            obj['Middleware_Tag']
            is_middleware = True
        except KeyError as detail:
            pass
    
        if is_middleware:
            # Create an object instance and store it
            instance = create_instance (obj)
            if instance != None:
                GameLogic.mwDict[obj] = instance
                logger.info("\tMiddleware '%s' found" % obj)
            else:
                return False
    
    # Will return true always (for the moment)
    return True


def check_dictionaries():
    """ Print the contents of the robot and component dictionaries."""
    logger.info("------------------------------------")
    logger.info("GameLogic has the following robots:")
    for obj, robot_instance in GameLogic.robotDict.items():
        logger.info("\tROBOT: '{0}'".format(obj))
        for component in robot_instance.components:
            logger.info ("\t\t- Component: '{0}'".format(component))

    logger.info ("GameLogic has the following external robots:")
    for obj, robot_position in GameLogic.externalRobotDict.items():
        logger.info ("\tROBOT: '{0}'".format(obj))

    logger.info ("GameLogic has the following components:")
    for obj, component_variables in GameLogic.componentDict.items():
        logger.info ("\tCOMPONENT: '{0}'".format(obj))

    logger.info ("GameLogic has the following modifiers:")
    for obj, modifier_variables in GameLogic.modifierDict.items():
        logger.info ("\tMODIFIER: '{0}'".format(obj))
        
    logger.info ("GameLogic has the following middlewares:")
    for obj, mw_variables in GameLogic.mwDict.items():
        logger.info ("\tMIDDLEWARE: '{0}'".format(obj))
        
    logger.info ("GameLogic has the following request managers:")
    for rqst_manager in GameLogic.morse_services._request_managers.keys():
        logger.info ("\tREQUEST MANAGER: '{0}'".format(rqst_manager))
        
    logger.info ("GameLogic has the following services:")
    for obj, service_variables in GameLogic.serviceDict.items():
        logger.info ("\tSERVICE: '{0}'".format(obj))

def get_class(module_name, class_name):
    """ Dynamically creates an instance of a Python class.
    """
    try:
        __import__(module_name)
    except ImportError as detail:
        logger.error ("Module not found: %s" % detail)
        return None
    module = sys.modules[module_name]
    # Create an instance of the object class
    try:
        klass = getattr(module, class_name)
    except AttributeError as detail:
        logger.error ("Module attribute not found: %s" % detail)
        return None
    return klass

def create_instance(obj, parent=None):
    """ Dynamically load a Python module and create an instance object
        of the class defined within. """
    # Read the path and class of the object from the Logic Properties
    source_file = obj['Path']
    module_name = re.sub('/', '.', source_file)
    logger.debug("Path to Component Class: %s" % module_name)
    klass = get_class(module_name, obj['Class'])
    return klass(obj, parent)

def create_mw(mw):
    """ Creates an instances of a middleware class.
    """
    modulename, classname = mw.rsplit('.', 1)
    klass = get_class(modulename, classname)
    return klass()

def get_components_of_type(classname):
    components = []
    for component in GameLogic.componentDict.values():
        logger.debug("Get component for class " + component.name() + ": " + component.__class__.__name__)
        if component.__class__.__name__ == classname:
            components.append(component)
    
    return components


def get_middleware_of_type(classname):
    for mw_instance in GameLogic.mwDict.values():
        if mw_instance.__class__.__name__ == classname:
            return mw_instance
    return None
    

def link_middlewares():
    """ Read the configuration script (inside the .blend file)
        and assign the correct middleware and options to each component. """
    try:
        component_list = component_config.component_mw
    except (AttributeError, NameError) as detail:
        # Exit gracefully if there are no modifiers specified
        logger.info ("No middleware section found in configuration file.")
        return True

    #for component_name, mw_data in component_list.items():
    for component_name, mw_list in component_list.items():
        # Get the instance of the object
        try:
            instance = GameLogic.componentDict[component_name]
        except KeyError as detail:
            logger.error ("Component listed in component_config.py not found in scene: {0}".format(detail))
            logger.error("""
    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    ERROR: your configuration file is not valid. Please
    check the name of your components and restart the
    simulation.
    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            """)
            return False

        # Do not configure middlewares for components that are external,
        #  that is, they are handled by another node.
        try:
            instance.robot_parent.blender_obj['Robot_Tag']
            # If the robot is external, it will have the 'External_Robot_Tag'
            #  instead, and this test will fail
        except KeyError as detail:
            # Skip the configuration of this component
            continue

        # If the list contains only strings, insert the list inside another one.
        # This is done for backwards compatibility with the previous
        #  syntax that allowed only one middleware per component
        if isinstance (mw_list[0], str):
            mw_list = [mw_list]

        # Register all mw's in the list
        for mw_data in mw_list:

            mw_name = mw_data[0]
            logger.info("Component: '%s' using middleware '%s'" % (component_name, mw_name))
            found = False
            missing_component = False
            
            # Look for the listed mw in the dictionary of active mw's
            for mw_obj, mw_instance in GameLogic.mwDict.items():
                logger.debug("Looking for '%s' in '%s'" % (mw_name, mw_obj))
                if mw_name in mw_obj:
                    found = True
                    # Make the middleware object take note of the component
                    break

            if not found:
                mw_instance = create_mw (mw_name)
                if mw_instance != None:
                    GameLogic.mwDict[mw_name] = mw_instance
                    logger.info("\tMiddleware '%s' created" % mw_name)
                else:
                    logger.warning("WARNING: There is no '%s' middleware object in the scene." % mw_name)
                    return False
            
            mw_instance.register_component(component_name, instance, mw_data)
            
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
            instance = GameLogic.componentDict[component_name]
        except KeyError as detail:
            try:
                scene = GameLogic.getCurrentScene()
                robot_obj = scene.objects[component_name]
                instance = GameLogic.robotDict[robot_obj]

            except KeyError as detail:
                logger.error("Component listed in component_config.py not found in scene: {0}".format(detail))
                logger.error("""
    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    ERROR: the component_services section of your
    configuration file is not valid. Please check the 
    name of your components and restart the simulation.
    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                """)
                return False

        for request_manager in request_manager_data:
            try:
                modulename, classname = request_manager.rsplit('.', 1)
            except ValueError:
                logger.error("You must specify the fully qualified name " + \
                             "of the request manager (eg: " + \
                             "morse.middleware.socket_request_manager.SocketRequestManager)")
                return False
            
            # Load required request managers
            if not GameLogic.morse_services.add_request_manager(request_manager):
                return False
            
            GameLogic.morse_services.register_request_manager_mapping(component_name, classname)
            instance.register_services()
            logger.info("Component: '%s' using middleware '%s' for services" % (component_name, classname))
    
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
        
        # In case the fully-qualified name of the request manager has been 
        # provided, keep only the class name
        request_manager_name = request_manager_name.split('.')[-1]
        
        for overlaid_name, overlay_name in overlays.items():
            modulename, classname = overlay_name.rsplit('.', 1)
            
            try:
                __import__(modulename)
            except ImportError as detail:
                logger.error("Module for overlay %s not found: %s." % (classname, detail))
                return False

            module = sys.modules[modulename]
            # Create an instance of the object class
            try:
                klass = getattr(module, classname)
            except AttributeError as detail:
                logger.error("Overlay not found: %s." % detail)
                return False

            try:
                overlaid_object = GameLogic.componentDict[overlaid_name]
            except KeyError:
                logger.error("Could not find the object to overlay: %s." % overlaid_name)
                return False

            # Instanciate the overlay, passing the overlaid object to
            # the constructor
            instance = klass(overlaid_object)
            GameLogic.morse_services.register_request_manager_mapping(instance.name(), request_manager_name)
            instance.register_services()
            GameLogic.overlayDict[overlay_name] = instance
            logger.info("Component '%s' overlaid with '%s' using middleware '%s' for services" % (overlaid_object.name(), overlay_name, request_manager_name))
    
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
            instance = GameLogic.componentDict[component_name]
        except KeyError as detail:
            logger.warning("Component listed in component_config.py not found in scene: {0}".format(detail))
            continue

        for mod_data in mod_list:
            modifier_name = mod_data[0]
            logger.info("Component: '%s' operated by '%s'" % (component_name, modifier_name))
            found = False
            # Look for the listed modifier in the dictionary of active modifier's
            for modifier_obj, modifier_instance in GameLogic.modifierDict.items():
                if modifier_name in modifier_obj:
                    found = True
                    break
                    
            if not found:
                modifier_instance = create_mw(modifier_name)
                if modifier_instance != None:
                    GameLogic.modifierDict[modifier_name] = modifier_instance
                    logger.info("\tModifier '%s' created" % modifier_name)
                else:
                    logger.warning("There is no '%s' modifier object in the scene." % modifier_name)
                    return False
            # Make the modifier object take note of the component
            modifier_instance.register_component(component_name, instance, mod_data)
    
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
        klass = get_class("morse.multinode.socket", "SocketNode")
    elif protocol == "hla":
        klass = get_class("morse.multinode.hla", "HLANode")

    try:
        node_name = multinode_config.node_config["node_name"]
        server_address = multinode_config.node_config["server_address"]
        server_port = int(multinode_config.node_config["server_port"])
    except (NameError, AttributeError) as detail:
        logger.warning("No node configuration found. Using default values for this simulation node.\n\tException: ", detail)
        node_name = os.uname()[1]
        server_address = "localhost"
        server_port = 65000

    logger.info ("This is node '%s'" % node_name)
    # Create the instance of the node class
    GameLogic.node_instance = klass(node_name, server_address, server_port,
            GameLogic)

def init(contr):
    """ General initialization of MORSE

    Here, all components, modifiers and middleware are initialized.
    """

    init_logging()

    logger.log(SECTION, 'PRE-INITIALIZATION')
    # Get the version of Python used
    # This is used to determine also the version of Blender
    GameLogic.pythonVersion = sys.version_info
    GameLogic.blenderVersion = bpy.app.version
    logger.info ("Python Version: %s.%s.%s" % GameLogic.pythonVersion[:3])
    logger.info ("Blender Version: %s.%s.%s" % GameLogic.blenderVersion)

    GameLogic.morse_initialised = False
    GameLogic.base_clock = time.clock()
    GameLogic.current_time = 0.0
    # Variable to keep trac of the camera being used
    GameLogic.current_camera_index = 0
    init_ok = True


    logger.log(SECTION, 'SUPERVISION SERVICES INITIALIZATION')
    init_ok = init_supervision_services()
    
    logger.log(SECTION, 'SCENE INITIALIZATION')
    init_ok = init_ok and create_dictionaries()
    init_ok = init_ok and link_services()
    init_ok = init_ok and add_modifiers()
    init_ok = init_ok and link_middlewares()
    init_ok = init_ok and load_overlays()

    if init_ok:
        logger.log(ENDSECTION, 'SCENE INITIALIZED')
        check_dictionaries()
        GameLogic.morse_initialised = True
    else:
        logger.critical('INITIALIZATION FAILED!')
        logger.info("Exiting now.")
        contr = GameLogic.getCurrentController()
        close_all(contr)
        quit(contr)

    if MULTINODE_SUPPORT:
        init_multinode()
    
    # Set the default value of the logic tic rate to 60
    #GameLogic.setLogicTicRate(60.0)
    #GameLogic.setPhysicsTicRate(60.0)

def init_logging():
    from morse.core.ansistrm import ColorizingStreamHandler
    from morse.core.logging import MorseFormatter
    # create logger
    logger = logging.getLogger('morse')
    logger.setLevel(logging.INFO)

    # create console handler and set level to debug
    ch = ColorizingStreamHandler()
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
    :py:module:`morse.core.supervision_services` 
    """
    GameLogic.morse_services = MorseServices()

    ###
    # First, load and map the socket request manager to the pseudo
    # 'simulation' component:
    try:
        if not GameLogic.morse_services.add_request_manager("morse.middleware.socket_request_manager.SocketRequestManager"):
            return False
        # The simulation mangement services always uses at least sockets for requests.
        GameLogic.morse_services.register_request_manager_mapping("simulation", "SocketRequestManager")
        GameLogic.morse_services.register_request_manager_mapping("communication", "SocketRequestManager")

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
                modulename, classname = request_manager.rsplit('.', 1)
            except ValueError:
                logger.error("You must specify the fully qualified name " + \
                             "of the request manager (eg: " + \
                             "morse.middleware.socket_request_manager.SocketRequestManager)")
                return False

            try:
                # Load required request managers
                if not GameLogic.morse_services.add_request_manager(request_manager):
                    return False

                GameLogic.morse_services.register_request_manager_mapping("simulation", classname)
                logger.info("Adding '{}' to the middlewares for simulation control".format(classname))
            except MorseServiceError as e:
                #...no request manager :-(
                logger.critical(str(e))
                logger.critical("SUPERVISION SERVICES INITIALIZATION FAILED")
                return False

    except (AttributeError, NameError, KeyError) as detail:
        # Nothing to declare: skip to the next step.
        pass


    ###
    # Services can be imported *only* after GameLogic.morse_services
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
        GameLogic.current_time = time.clock() - GameLogic.base_clock
    except AttributeError as detail:
        # If the 'base_clock' variable is not defined, there probably was
        #  a problem while doing the init, so we'll abort the simulation.
        logger.critical("""
    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    ERROR: the initialisation of the simulation
    was not correctly done.
    Check the terminal for error messages, and report
    them on the morse-dev@laas.fr mailing list.
    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        """)
        quit(contr)

    if hasattr(GameLogic, "morse_services"):
        # let the service managers process their inputs/outputs
        GameLogic.morse_services.process()
    
    if MULTINODE_SUPPORT:
        # Register the locations of all the robots handled by this node
        GameLogic.node_instance.synchronize()


def switch_camera(contr):
    """ Cycle through the cameras in the scene during the game.
    """
    sensor = contr.sensors['F9_KEY']
    # Activate only once for each key press
    if sensor.positive and sensor.triggered:
        scene = GameLogic.getCurrentScene()
        index = GameLogic.current_camera_index
        next_camera = scene.cameras[index]
        scene.active_camera = next_camera
        logger.info("Showing view from camera: '%s'" % next_camera.name)
        # Update the index for the next call
        index = (index + 1) % len(scene.cameras)
        GameLogic.current_camera_index = index


def close_all(contr):
    """ Close the open communication channels from middlewares
    Call the destructors of all component instances. This should also call
    the methods to close middlewares
    """
    logger.log(ENDSECTION, 'COMPONENTS FINALIZATION')
    # Force the deletion of the sensor objects
    if hasattr(GameLogic, 'componentDict'):
        for obj, component_instance in GameLogic.componentDict.items():
            del obj

    # Force the deletion of the robot objects
    if hasattr(GameLogic, 'robotDict'):
        for obj, robot_instance in GameLogic.robotDict.items():
            del obj

    logger.log(ENDSECTION, 'CLOSING REQUEST MANAGERS...')
    del GameLogic.morse_services

    logger.log(ENDSECTION, 'CLOSING MIDDLEWARES...')
    # Force the deletion of the middleware objects
    if hasattr(GameLogic, 'mwDict'):
        for obj, mw_instance in GameLogic.mwDict.items():
            if mw_instance:
                mw_instance.cleanup()
                import gc # Garbage Collector
                logger.debug("At closing time, %s has %s references" % (mw_instance, gc.get_referents(mw_instance)))
                del obj

    if MULTINODE_SUPPORT:
        logger.log(ENDSECTION, 'CLOSING MULTINODE...')
        GameLogic.node_instance.finalize()


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
    for b_obj, state in GameLogic.blender_objects.items():
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
