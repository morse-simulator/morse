import sys
import re
import time
import bpy
import GameLogic
import Rasterizer

# The file component_config.py is at the moment included
#  in the .blend file of the scene
try:
    import component_config
    
    # The service management
    from morse.core.services import MorseServices

except ImportError as detail:
    print ("WARNING: ", detail, ". No middlewares will be configured")


MULTINODE_SUPPORT = False
# The file scene_config.py is at the moment included
#  in the .blend file of the scene
# Used to setup the multinode information
try:
    import scene_config
    
    # Multi-node simulation
    import morse.core.multinode
    MULTINODE_SUPPORT = True
    
except ImportError as detail:
    print ("INFO: No multi-node scene configuration file found." + \
           " Multi-node support disabled.")

from morse.core.exceptions import MorseServiceError


# Create a list of the robots in the scene
def create_dictionaries ():
    """Creation of a list of all the robots and components in the scene.
       Uses the properties of the objects to determine what they are."""

    # Create a dictionary with the middlewares used
    if not hasattr(GameLogic, 'blender_objects'):
        GameLogic.blender_objects = {}

    # Create a dictioary of the components in the scene
    if not hasattr(GameLogic, 'componentDict'):
        GameLogic.componentDict = {}

    # Create a dictionary of the robots in the scene
    if not hasattr(GameLogic, 'robotDict'):
        GameLogic.robotDict = {}

    # Create a dictionary of the external robots in the scene
    # Used for the multi-node simulation
    if not hasattr(GameLogic, 'externalRobotDict'):
        GameLogic.externalRobotDict = {}

    # Create a dictionary with the modifiers
    if not hasattr(GameLogic, 'modifierDict'):
        GameLogic.modifierDict = {}

    # Create a dictionary with the middlewares used
    if not hasattr(GameLogic, 'mwDict'):
        GameLogic.mwDict = {}

    # Create a dictionary with the middlewares used
    if not hasattr(GameLogic, 'serviceDict'):
        GameLogic.serviceDict = {}

    scene = GameLogic.getCurrentScene()

    # Store the position and orientation of all objects
    for obj in scene.objects:
        if obj.parent == None:
            import mathutils
            pos = mathutils.Vector(obj.worldPosition)
            ori = mathutils.Matrix(obj.worldOrientation)
            GameLogic.blender_objects[obj] = [pos, ori]

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
    
    if not GameLogic.robotDict: # No robot!
        print("""
            !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            ERROR: no robot in your simulation!
            
            Do not forget that components _must_ belong to a
            robot (you can not have free objects)
            !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            """)
        return False
    
    # Get the robot and its instance
    for obj, robot_instance in GameLogic.robotDict.items():
        # Create an empty list for the components of this robot
        robot_instance.components = []
        for child in obj.childrenRecursive:
            try:
                # Look for the components tagged as such
                child['Component_Tag']
                robot_instance.components.append (child)

                # Create an instance of the component class
                #  and add it to the component list of GameLogic
                instance = create_instance (child, robot_instance)
                if instance != None:
                    GameLogic.componentDict[child.name] = instance
                else:
                    return False

            except KeyError:
                pass
    
    # Check we have no 'free' component (they all must belong to a robot)
    for obj in scene.objects:
        try:
            obj['Component_Tag']
            if obj.name not in GameLogic.componentDict.keys():
                print("""
                    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                    ERROR: the component """ + obj.name + """ do not
                    belong to any robot: you need to fix that by 
                    parenting it to a robot.                    
                    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                    """)
                return False
        except KeyError as detail:
            pass
            
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
                print ("\tMiddleware '%s' found" % obj)
            else:
                return False

    # Will return true always (for the moment)
    return True


def check_dictionaries():
    """ Print the contents of the robot and component dictionaries."""
    print ("------------------------------------")
    print ("GameLogic has the following robots:")
    for obj, robot_instance in GameLogic.robotDict.items():
        print ("\tROBOT: '{0}'".format(obj))
        for component in robot_instance.components:
            print ("\t\t- Component: '{0}'".format(component))

    print ("GameLogic has the following external robots:")
    for obj, robot_position in GameLogic.externalRobotDict.items():
        print ("\tROBOT: '{0}'".format(obj))

    print ("\nGameLogic has the following components:")
    for obj, component_variables in GameLogic.componentDict.items():
        print ("\tCOMPONENT: '{0}'".format(obj))

    print ("\nGameLogic has the following modifiers:")
    for obj, modifier_variables in GameLogic.modifierDict.items():
        print ("\tMODIFIER: '{0}'".format(obj))

    print ("\nGameLogic has the following middlewares:")
    for obj, mw_variables in GameLogic.mwDict.items():
        print ("\tMIDDLEWARE: '{0}'".format(obj))

    print ("\nGameLogic has the following services:")
    for obj, service_variables in GameLogic.serviceDict.items():
        print ("\tSERVICE: '{0}'".format(obj))


def create_instance(obj, parent=None):
    """ Dynamically load a Python module and create an instance object
        of the class defined within. """
    # Read the path and class of the object from the Logic Properties
    source_file = obj['Path']
    module_name = re.sub('/', '.', source_file)
    #print ("Path to Component Class: %s" % module_name)
    # Import the module containing the class
    try:
        __import__(module_name)
    except ImportError as detail:
        print ("ERROR: Module not found: %s" % detail)
        return None
    module = sys.modules[module_name]
    # Create an instance of the object class
    try:
        klass = getattr(module, obj['Class'])
    except AttributeError as detail:
        print ("ERROR: Module attribute not found: %s" % detail)
        return None
    instance = klass(obj, parent)

    return instance


def link_middlewares():
    """ Read the configuration script (inside the .blend file)
        and assign the correct middleware and options to each component. """
    try:
        component_list = component_config.component_mw
    except AttributeError as detail:
        # Exit gracefully if there are no modifiers specified
        print ("[INFO] No middleware section found in configuration file.")
        return True

    #for component_name, mw_data in component_list.items():
    for component_name, mw_list in component_list.items():
        # Get the instance of the object
        try:
            instance = GameLogic.componentDict[component_name]
        except KeyError as detail:
            print ("Component listed in component_config.py not found in scene: {0}".format(detail))
            print("""
            !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            ERROR: your configuration file is not valid. Please
            check the name of your components and restart the
            simulation.
            !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            """)
            return False

        # If the list contains only strings, insert the list inside another one.
        # This is done for backwards compatibility with the previous
        #  syntax that allowed only one middleware per component
        if isinstance (mw_list[0], str):
            mw_list = [mw_list]

        # Register all mw's in the list
        for mw_data in mw_list:

            mw_name = mw_data[0]
            print ("Component: '%s' using middleware '%s'" % (component_name, mw_name))
            found = False
            missing_component = False
            # Look for the listed mw in the dictionary of active mw's
            for mw_obj, mw_instance in GameLogic.mwDict.items():
                #print("Looking for '%s' in '%s'" % (mw_name, mw_obj.name))
                if mw_name in mw_obj.name:
                    found = True
                    # Make the middleware object take note of the component
                    mw_instance.register_component(component_name, instance, mw_data)
                    break

            if not found:
                print ("WARNING: There is no '%s' middleware object in the scene." % mw_name)

    # Will return true always (for the moment)
    return True


def link_services():
    """ Read the configuration script (inside the .blend file)
        and assign the correct service handlers and options to each component. """
    try:
        component_list = component_config.component_service
    except AttributeError as detail:
        # Exit gracefully if there are no services specified
        print ("[INFO] No service section found in configuration file.")
        return True

    for component_name, request_manager_data in component_list.items():
        # Get the instance of the object
        try:
            instance = GameLogic.componentDict[component_name]
        except KeyError as detail:
            try:
                scene = GameLogic.getCurrentScene()
                robot_obj = scene.objects[component_name]
                instance = GameLogic.robotDict[robot_obj]

            except KeyError as detail:
                print ("Component listed in component_config.py not found in scene: {0}".format(detail))
                print("""
                !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                ERROR: the component_services section of your
                configuration file is not valid. Please check the 
                name of your components and restart the simulation.
                !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                """)
                return False

        for request_manager in request_manager_data:
            modulename, classname = request_manager.rsplit('.', 1)
            
            # Load required request managers
            if not GameLogic.morse_services.add_request_manager(request_manager):
                return False
            
            GameLogic.morse_services.register_request_manager_mapping(component_name, classname)
            instance.register_services()
            print ("Component: '%s' using middleware '%s' for services" % (component_name, classname))
    
    return True


def add_modifiers():
    """ Read the configuration script (inside the .blend file)
        and assign the correct data modifiers to each component. """
    try:
        component_list = component_config.component_modifier
    except AttributeError as detail:
        # Exit gracefully if there are no modifiers specified
        print ("[INFO] No modifiers section found in configuration file")
        return True

    for component_name, mod_list in component_list.items():
        for mod_data in mod_list:
            modifier_name = mod_data[0]
            print ("Component: '%s' operated by '%s'" % (component_name, modifier_name))
            found = False
            # Look for the listed modifier in the dictionary of active modifier's
            for modifier_obj, modifier_instance in GameLogic.modifierDict.items():
                if modifier_name in modifier_obj.name:
                    found = True
                    # Get the instance of the object
                    try:
                        instance = GameLogic.componentDict[component_name]
                    except KeyError as detail:
                        print ("Component listed in component_config.py not found in scene: {0}".format(detail))
                        continue

                    # Make the modifier object take note of the component
                    modifier_instance.register_component(component_name, instance, mod_data)

            if not found:
                print ("There is no '%s' modifier object in the scene." % modifier_name)
    
    return True


def init(contr):
    """ General initialization of MORSE

    Here, all components, modifiers and middleware are initialized.
    """
    
    if MULTINODE_SUPPORT:
        # Configuration for the multi-node simulation
        try:
            node_name = scene_config.node_config["node_name"]
            server_address = scene_config.node_config["server_address"]
            server_port = scene_config.node_config["server_port"]
        #except NameError as detail:
            #print ("No definition file for 'multi-node' instance found. Using only single node")
        #except AttributeError as detail:
        except (NameError, AttributeError) as detail:
            print ("WARNING: No node configuration found. Using default values for this simulation node.\n\tException: ", detail)
            node_name = "temp_name"
            server_address = "localhost"
            server_port = 65000
        GameLogic.node_instance = morse.core.multinode.SimulationNodeClass(node_name, server_address, server_port)


    print ('\n######## SCENE INITIALIZATION ########')
    # Get the version of Python used
    # This is used to determine also the version of Blender
    GameLogic.pythonVersion = sys.version_info
    GameLogic.blenderVersion = bpy.app.version
    print ("Python Version: %s.%s.%s" % GameLogic.pythonVersion[:3])
    print ("Blender Version: {0}".format(GameLogic.blenderVersion))

    GameLogic.morse_initialised = False
    GameLogic.base_clock = time.clock()
    GameLogic.current_time = 0.0
    # Variable to keep trac of the camera being used
    GameLogic.current_camera_index = 0
    init_ok = True


    print ('\n######## SUPERVISION SERVICES INITIALIZATION ########')
    init_ok = init_supervision_services()
    
    print ('########  COMPONENT DICTIONARY INITIALISATION ######## ')
    init_ok = init_ok and create_dictionaries()
    init_ok = init_ok and add_modifiers()
    init_ok = init_ok and link_middlewares()
    init_ok = init_ok and link_services()

    if init_ok:
        print ('########  COMPONENT DICTIONARY INITIALIZED ######## ')
        check_dictionaries()
        GameLogic.morse_initialised = True
    else:
        print ('########  INITIALISATION FAILED!!! ######## ')
        print ("Exiting the simulation!")
        contr = GameLogic.getCurrentController()
        close_all(contr)
        quit(contr)


    # Set the default value of the logic tic rate to 60
    #GameLogic.setLogicTicRate(60.0)
    #GameLogic.setPhysicsTicRate(60.0)


def init_supervision_services():
    """ This method first loads the socket service manager, map the virtual
    'supervision' component to it, and register all supervision services
    declared in morse.core.supervision_services;
    """
    GameLogic.morse_services = MorseServices()

    try:
        if not GameLogic.morse_services.add_request_manager("morse.middleware.socket_request_manager.SocketRequestManager"):
            return False
        
        # The simulation 'supervision' always uses at least sockets for requests.
        GameLogic.morse_services.register_request_manager_mapping("simulation", "SocketRequestManager")

        # Services can be imported *only* after GameLogic.morse_services
        # has been created. Else @service won't know where to register the RPC
        # callbacks.
        import morse.core.supervision_services

        print("########  SUPERVISION SERVICES INITIALIZED ######## ")
    except MorseServiceError as e:
        #...no request manager :-(
        print("WARNING: " + str(e))
        print("########  SUPERVISION SERVICES INITIALIZATION FAILED ######## ")
        return False
    
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
        print("""
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
        GameLogic.node_instance.synchronise_world(GameLogic)


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
        print ("Showing view from camera: '%s'" % next_camera.name)
        # Update the index for the next call
        index = (index + 1) % len(scene.cameras)
        GameLogic.current_camera_index = index


def close_all(contr):
    print ('######### FINALIZING COMPONENTS... ########')
    # Force the deletion of the sensor objects
    if hasattr(GameLogic, 'componentDict'):
        for obj, component_instance in GameLogic.componentDict.items():
            del obj

    # Force the deletion of the robot objects
    if hasattr(GameLogic, 'robotDict'):
        for obj, robot_instance in GameLogic.robotDict.items():
            del obj

    print ('######### CLOSING REQUEST MANAGERS... ########')
    del GameLogic.morse_services

    print ('######### CLOSING MIDDLEWARES... ########')
    # Force the deletion of the middleware objects
    if hasattr(GameLogic, 'mwDict'):
        for obj, mw_instance in GameLogic.mwDict.items():
            if mw_instance:
                mw_instance.cleanup()
                #import gc
                #print ("At closing time, %s has %s references" % (mw_instance, gc.get_referents(mw_instance)))
                del obj

    if MULTINODE_SUPPORT:
        print ('######### CLOSING MULTINODE... ########')
        GameLogic.node_instance.finish_node()


def finish(contr):
    """Close the open ports."""
    sensor = contr.sensors['ESC_KEY']

    #execute only when the ESC key is released (if we don't test that,
    #the code get executed two time, when pressed, and when released)
    if not sensor.positive and sensor.triggered:
        close_all(contr)
        quit(contr)

def restart(contr):
    """Close the open ports."""
    sensor = contr.sensors['F11_KEY']

    # Execute only when the F11 key is released (if we don't test that,
    #  the code get executed two times, when pressed, and when released)
    if not sensor.positive and sensor.triggered:

        print ("### Replacing everything ###")
        reset_objects(contr)
        return

def quit(contr):
    print ('######### EXITING SIMULATION ########')
        
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

        #print ("%s goes to %s" % (b_obj, state[0]))
        b_obj.worldPosition = state[0]
        b_obj.worldOrientation = state[1]
        # Reset physics simulation
        b_obj.restoreDynamics()
