import logging; logger = logging.getLogger("morse." + __name__)
from morse.helpers.morse_logging import SECTION, ENDSECTION
import sys
import os
import re
import time
import bpy
import bge

import morse
morse.running_in_blender = True

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
        #  and add it to the component list of bge.logic
        instance = create_instance (child, robot_instance)
        if instance != None:
            bge.logic.componentDict[child.name] = instance
        else:
            logger.error("""
    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    INITIALIZATION ERROR: the component '""" + obj.name + """' could not
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
    if not hasattr(bge.logic, 'blender_objects'):
        bge.logic.blender_objects = {}

    # Create a dictionary of the components in the scene
    if not hasattr(bge.logic, 'componentDict'):
        bge.logic.componentDict = {}

    # Create a dictionary of the robots in the scene
    if not hasattr(bge.logic, 'robotDict'):
        bge.logic.robotDict = {}

    # Create a dictionary of the external robots in the scene
    # Used for the multi-node simulation
    if not hasattr(bge.logic, 'externalRobotDict'):
        bge.logic.externalRobotDict = {}

    # Create a dictionnary with the passive, but interactive (ie, with an
    # 'Object' property) objects in the scene.
    if not hasattr(bge.logic, 'passiveObjectsDict'):
        bge.logic.passiveObjectsDict = {}

    # Create a dictionary with the modifiers
    if not hasattr(bge.logic, 'modifierDict'):
        bge.logic.modifierDict = {}

    # Create a dictionary with the middlewares used
    if not hasattr(bge.logic, 'mwDict'):
        bge.logic.mwDict = {}

    # Create a dictionary with the service used
    if not hasattr(bge.logic, 'serviceDict'):
        bge.logic.serviceDict = {}

    # Create a dictionnary with the overlaid used
    if not hasattr(bge.logic, 'overlayDict'):
        bge.logic.overlayDict = {}

    scene = bge.logic.getCurrentScene()

    # Store the position and orientation of all objects
    for obj in scene.objects:
        if obj.parent == None:
            import mathutils
            pos = mathutils.Vector(obj.worldPosition)
            ori = mathutils.Matrix(obj.worldOrientation)
            bge.logic.blender_objects[obj] = [pos, ori]

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
            bge.logic.passiveObjectsDict[obj] = details
            logger.info("Added {name} as a {graspable}active object".format(
                                 name = details['label'],
                                 graspable = "graspable " if details['graspable'] else ""))

    if not bge.logic.passiveObjectsDict:
        logger.info("No passive objects in the scene.")

    # Get the robots
    for obj in scene.objects:
        try:
            obj['Robot_Tag']
            # Create an object instance and store it
            instance = create_instance (obj)
            if instance != None:
                bge.logic.robotDict[obj] = instance
            else:
                return False
        except KeyError as detail:
            pass
        try:
            obj['External_Robot_Tag']
            # Create an object instance and store it
            instance = create_instance (obj)
            if instance != None:
                bge.logic.externalRobotDict[obj] = instance
            else:
                return False
        except KeyError as detail:
            pass
    
    if not (bge.logic.robotDict or bge.logic.externalRobotDict): # No robot!
        logger.error("""
    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    INITIALIZATION ERROR: no robot in your simulation!
    
    Do not forget that components _must_ belong to a
    robot (you can not have free objects)
    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            """)
        return False

    
    # Get the robot and its instance
    for obj, robot_instance in bge.logic.robotDict.items():
        if not _associate_child_to_robot(obj, robot_instance, False):
            return False
    
    # Get the external robot and its instance
    for obj, robot_instance in bge.logic.externalRobotDict.items():
        if not _associate_child_to_robot(obj, robot_instance, True):
            return False
  
    # Check we have no 'free' component (they all must belong to a robot)
    for obj in scene.objects:
        try:
            obj['Component_Tag']
            if obj.name not in bge.logic.componentDict.keys():
                logger.error("""
    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    INITIALIZATION ERROR: the component '""" + obj.name + """' does not
    belong to any robot: you need to fix that by 
    parenting it to a robot.                    
    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                    """)
                return False
        except KeyError as detail:
            pass
    
    # Will return true always (for the moment)
    return True


def check_dictionaries():
    """ Print the contents of the robot and component dictionaries."""
    logger.info("------------------------------------")
    logger.info("bge.logic has the following robots:")
    for obj, robot_instance in bge.logic.robotDict.items():
        logger.info("\tROBOT: '{0}'".format(obj))
        for component in robot_instance.components:
            logger.info ("\t\t- Component: '{0}'".format(component))

    logger.info ("bge.logic has the following external robots:")
    for obj, robot_position in bge.logic.externalRobotDict.items():
        logger.info ("\tROBOT: '{0}'".format(obj))

    logger.info ("bge.logic has the following components:")
    for obj, component_variables in bge.logic.componentDict.items():
        logger.info ("\tCOMPONENT: '{0}'".format(obj))

    logger.info ("bge.logic has the following modifiers:")
    for obj, modifier_variables in bge.logic.modifierDict.items():
        logger.info ("\tMODIFIER: '{0}'".format(obj))
        
    logger.info ("bge.logic has the following middlewares:")
    for obj, mw_variables in bge.logic.mwDict.items():
        logger.info ("\tMIDDLEWARE: '{0}'".format(obj))
        
    logger.info ("bge.logic has the following request managers:")
    for rqst_manager in bge.logic.morse_services._request_managers.keys():
        logger.info ("\tREQUEST MANAGER: '{0}'".format(rqst_manager))
        
    logger.info ("bge.logic has the following services:")
    for obj, service_variables in bge.logic.serviceDict.items():
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
    if klass != None:
        return klass(obj, parent)
    else:
        return None

def create_mw(mw):
    """ Creates an instances of a middleware class.
    """
    modulename, classname = mw.rsplit('.', 1)
    klass = get_class(modulename, classname)
    if klass != None:
        return klass()
    else:
        logger.error("""
    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    INITIALIZATION ERROR: Middleware '""" + modulename + """'
    module could not be found!
    
    Make sure you selected the required middleware for
    install from the cmake configuration.
    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        """)

def get_components_of_type(classname):
    components = []
    for component in bge.logic.componentDict.values():
        logger.debug("Get component for class " + component.name() + ": " + component.__class__.__name__)
        if component.__class__.__name__ == classname:
            components.append(component)
    
    return components


def get_middleware_of_type(classname):
    for mw_instance in bge.logic.mwDict.values():
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
            instance = bge.logic.componentDict[component_name]
        except KeyError as detail:
            logger.error ("Component listed in component_config.py not found in scene: {0}".format(detail))
            logger.error("""
    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    INITIALIZATION ERROR: your configuration file is not valid. Please
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
            for mw_obj, mw_instance in bge.logic.mwDict.items():
                logger.debug("Looking for '%s' in '%s'" % (mw_name, mw_obj))
                if mw_name in mw_obj:
                    found = True
                    # Make the middleware object take note of the component
                    break

            if not found:
                mw_instance = create_mw (mw_name)
                if mw_instance != None:
                    bge.logic.mwDict[mw_name] = mw_instance
                    logger.info("\tMiddleware '%s' created" % mw_name)
                else:
                    logger.error("""
    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    INITIALIZATION ERROR: Middleware '""" + mw_name + """'
    module could not be found!
    
    Could not import modules necessary for the selected
    middleware. Check that they can be found inside
    your PYTHONPATH variable.
    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                    """)
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
            instance = bge.logic.componentDict[component_name]
        except KeyError as detail:
            try:
                scene = bge.logic.getCurrentScene()
                robot_obj = scene.objects[component_name]
                instance = bge.logic.robotDict[robot_obj]

            except KeyError as detail:
                logger.error("Component listed in component_config.py not found in scene: {0}".format(detail))
                logger.error("""
    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    INITIALIZATION ERROR: the component_services section of your
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
            if not bge.logic.morse_services.add_request_manager(request_manager):
                return False
            
            bge.logic.morse_services.register_request_manager_mapping(component_name, classname)
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
                overlaid_object = bge.logic.componentDict[overlaid_name]
            except KeyError:
                logger.error("Could not find the object to overlay: %s." % overlaid_name)
                return False

            # Instanciate the overlay, passing the overlaid object to
            # the constructor
            instance = klass(overlaid_object)
            bge.logic.morse_services.register_request_manager_mapping(instance.name(), request_manager_name)
            instance.register_services()
            bge.logic.overlayDict[overlay_name] = instance
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
            instance = bge.logic.componentDict[component_name]
        except KeyError as detail:
            logger.warning("Component listed in component_config.py not found in scene: {0}".format(detail))
            continue

        for mod_data in mod_list:
            modifier_name = mod_data[0]
            logger.info("Component: '%s' operated by '%s'" % (component_name, modifier_name))
            found = False
            # Look for the listed modifier in the dictionary of active modifier's
            for modifier_obj, modifier_instance in bge.logic.modifierDict.items():
                if modifier_name in modifier_obj:
                    found = True
                    break
                    
            if not found:
                modifier_instance = create_mw(modifier_name)
                if modifier_instance != None:
                    bge.logic.modifierDict[modifier_name] = modifier_instance
                    logger.info("\tModifier '%s' created" % modifier_name)
                else:
                    logger.error("""
    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    INITIALIZATION ERROR: Modifier '""" + modifier_name + """'
    module could not be found!
    
    Could not import modules necessary for the selected
    modifier. Check that they can be found inside
    your PYTHONPATH variable.
    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                    """)
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
        server_address = multinode_config.node_config["server_address"]
        server_port = int(multinode_config.node_config["server_port"])
    except (NameError, AttributeError) as detail:
        logger.warning("No node configuration found. Using default values for this simulation node.\n\tException: ", detail)
        server_address = "localhost"
        server_port = 65000

    try:
        node_name = multinode_config.node_config["node_name"]
    except (NameError, AttributeError) as detail:
        logger.warning("No node name defined. Using host name.\n\tException: ", detail)
        node_name = os.uname()[1]

    logger.info ("This is node '%s'" % node_name)
    # Create the instance of the node class
    bge.logic.node_instance = klass(node_name, server_address, server_port,
            bge.logic)

def init(contr):
    """ General initialization of MORSE

    Here, all components, modifiers and middleware are initialized.
    """
    
    init_logging()

    logger.log(SECTION, 'PRE-INITIALIZATION')
    # Get the version of Python used
    # This is used to determine also the version of Blender
    bge.logic.pythonVersion = sys.version_info
    bge.logic.blenderVersion = bpy.app.version
    logger.info ("Python Version: %s.%s.%s" % bge.logic.pythonVersion[:3])
    logger.info ("Blender Version: %s.%s.%s" % bge.logic.blenderVersion)
    logger.info ("PID: %d" % os.getpid())

    bge.logic.morse_initialised = False
    bge.logic.base_clock = time.clock()
    bge.logic.current_time = 0.0
    # Variable to keep trac of the camera being used
    bge.logic.current_camera_index = 0
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
        bge.logic.morse_initialised = True
    else:
        logger.critical('INITIALIZATION FAILED!')
        logger.info("Exiting now.")
        contr = bge.logic.getCurrentController()
        close_all(contr)
        quit(contr)

    if MULTINODE_SUPPORT:
        init_multinode()
    
    # Set the default value of the logic tic rate to 60
    #bge.logic.setLogicTicRate(60.0)
    #bge.logic.setPhysicsTicRate(60.0)

def init_logging():
    
    if "with-colors" in sys.argv:
        from morse.core.ansistrm import ColorizingStreamHandler
        if "with-xmas-colors" in sys.argv:
            ch = ColorizingStreamHandler(scheme = "xmas")
        elif "with-reverse-colors" in sys.argv:
            ch = ColorizingStreamHandler(scheme = "dark")
        else:
            ch = ColorizingStreamHandler()
        
    else:
        ch = logging.StreamHandler()
    
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
    :py:module:`morse.core.supervision_services` 
    """
    bge.logic.morse_services = MorseServices()

    ###
    # First, load and map the socket request manager to the pseudo
    # 'simulation' component:
    try:
        if not bge.logic.morse_services.add_request_manager("morse.middleware.socket_request_manager.SocketRequestManager"):
            return False
        # The simulation mangement services always uses at least sockets for requests.
        bge.logic.morse_services.register_request_manager_mapping("simulation", "SocketRequestManager")
        bge.logic.morse_services.register_request_manager_mapping("communication", "SocketRequestManager")

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
                if not bge.logic.morse_services.add_request_manager(request_manager):
                    return False

                bge.logic.morse_services.register_request_manager_mapping("simulation", classname)
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
    # Services can be imported *only* after bge.logic.morse_services
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
        bge.logic.current_time = time.clock() - bge.logic.base_clock
    except AttributeError as detail:
        # If the 'base_clock' variable is not defined, there probably was
        #  a problem while doing the init, so we'll abort the simulation.
        logger.critical("""
    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    INITIALIZATION ERROR: failure during initialization
    of the simulator.
    
    Check the terminal for error messages, and report
    them on the morse-dev@laas.fr mailing list.
    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        """)
        quit(contr)

    if hasattr(bge.logic, "morse_services"):
        # let the service managers process their inputs/outputs
        bge.logic.morse_services.process()
    
    if MULTINODE_SUPPORT:
        # Register the locations of all the robots handled by this node
        bge.logic.node_instance.synchronize()


def switch_camera(contr):
    """ Cycle through the cameras in the scene during the game.
    """
    sensor = contr.sensors['F9_KEY']
    # Activate only once for each key press
    if sensor.positive and sensor.triggered:
        scene = bge.logic.getCurrentScene()
        index = bge.logic.current_camera_index
        next_camera = scene.cameras[index]
        scene.active_camera = next_camera
        logger.info("Showing view from camera: '%s'" % next_camera.name)
        # Disable mouse cursor for Human camera
        if next_camera.name == "Human_Camera":
            bge.logic.mouse.visible = False
        else:
            bge.logic.mouse.visible = True
        # Update the index for the next call
        index = (index + 1) % len(scene.cameras)
        bge.logic.current_camera_index = index


def close_all(contr):
    """ Close the open communication channels from middlewares
    Call the destructors of all component instances. This should also call
    the methods to close middlewares
    """
    logger.log(ENDSECTION, 'COMPONENTS FINALIZATION')
    # Force the deletion of the sensor objects
    if hasattr(bge.logic, 'componentDict'):
        for obj, component_instance in bge.logic.componentDict.items():
            del obj

    # Force the deletion of the robot objects
    if hasattr(bge.logic, 'robotDict'):
        for obj, robot_instance in bge.logic.robotDict.items():
            del obj

    logger.log(ENDSECTION, 'CLOSING REQUEST MANAGERS...')
    del bge.logic.morse_services

    logger.log(ENDSECTION, 'CLOSING MIDDLEWARES...')
    # Force the deletion of the middleware objects
    if hasattr(bge.logic, 'mwDict'):
        for obj, mw_instance in bge.logic.mwDict.items():
            if mw_instance:
                mw_instance.cleanup()
                import gc # Garbage Collector
                logger.debug("At closing time, %s has %s references" % (mw_instance, gc.get_referents(mw_instance)))
                del obj

    if MULTINODE_SUPPORT:
        logger.log(ENDSECTION, 'CLOSING MULTINODE...')
        bge.logic.node_instance.finalize()


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
    for b_obj, state in bge.logic.blender_objects.items():
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
