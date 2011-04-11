import sys
import re
import time
import GameLogic
import Rasterizer

# Import this library to recover the Python version
import platform

# The file component_config.py is at the moment included
#  in the .blend file of the scene
try:
    import component_config
except ImportError as detail:
    print ("WARNING: ", detail, ". No middlewares will be configured")

# The service management
from morse.core.services import MorseServices

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
        except KeyError:
            pass
            #sys.exc_clear()    # Clears the last exception thrown
                                # Does not work in Python 3

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
                #sys.exc_clear()    # Clears the last exception thrown
                                    # Does not work in Python 3

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
            #sys.exc_clear()    # Clears the last exception thrown
                                # Does not work in Python 3


    # Get the middlewares
    for obj in scene.objects:
        is_middleware = False
        try:
            obj['Middleware_Tag']
            is_middleware = True
        except KeyError as detail:
            pass
            #sys.exc_clear()    # Clears the last exception thrown
                                # Does not work in Python 3
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
        print ("ERROR: The 'component_mw' dictionary can not be found in your configuration file.")
        return False

    for component_name, mw_data in component_list.items():
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
        print ("WARNING: The 'component_service' dictionary can not be found in your configuration file.")
        return False

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
                ERROR: your configuration file is not valid. Please
                check the name of your components and restart the
                simulation.
                !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                """)
                return False

        request_manager_name = request_manager_data[0]
        GameLogic.morse_services.register_request_manager_mapping(component_name, request_manager_name)
        instance.register_services()
        print ("Component: '%s' using middleware '%s' for services" % (component_name, request_manager_name))
        """
        found = False
        missing_component = False
        # Look for the listed service in the dictionary of active service's
        for mw_obj, mw_instance in GameLogic.mwDict.items():
            #print("Looking for '%s' in '%s'" % (mw_name, service_obj.name))
            if service_name in mw_obj.name:
                found = True
                # Make the service object take note of the component
                # TODO: Check that this works properly
                GameLogic.morse_services.register_request_manager_mapping(component_name, service_data[0])
                #service_instance.register_component(component_name, instance, service_data)
                break
                
        if not found:
            print ("WARNING: There is no '%s' service object in the scene." % service_name)
        """

    # Will return true always (for the moment)
    return True



def add_modifiers():
    """ Read the configuration script (inside the .blend file)
        and assign the correct data modifiers to each component. """
    try:
        component_list = component_config.component_modifier
    except AttributeError as detail:
        # Exit gracefully if there are no modifiers specified
        print ("No modifiers found in configuration file")
        return False

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

    # Will return true always (for the moment)
    return True

def init(contr):
    """ General initialization of MORSE

    Here, all components, modifiers and middleware are initialized.
    """

    print ('\n######## SERVICES INITIALIZATION ########')
    init_services()

    print ('\n######## SCENE INITIALIZATION ########')
    # Get the version of Python used, according to the pythonpath
    # This is used to determine also the version of Blender
    python_version = platform.python_version()
    print ("Python Version: {0}".format(python_version))
    # Chop the version to only 3 chars: #.#  and convert to a number
    GameLogic.pythonVersion = float(python_version[:3])
    GameLogic.morse_initialised = False
    GameLogic.base_clock = time.clock()
    GameLogic.current_time = 0.0
    # Variable to keep trac of the camera being used
    GameLogic.current_camera_index = 0
    init_ok = True

    print ('======== COMPONENT DICTIONARY INITIALISATION =======')
    init_ok = create_dictionaries()
    init_ok = init_ok and add_modifiers()
    init_ok = init_ok and link_middlewares()
    link_services()

    if init_ok:
        print ('======= COMPONENT DICTIONARY INITIALISED =======')
        check_dictionaries()
        GameLogic.morse_initialised = True
    else:
        print ('======= INITIALISATION FAILED!!! =======')
        print ("Exiting the simulation!")
        contr = GameLogic.getCurrentController()
        close_all(contr)


    # Set the default value of the logic tic rate to 60
    #GameLogic.setLogicTicRate(60.0)
    #GameLogic.setPhysicsTicRate(60.0)

    #Display the mouse in the simulator
    #Rasterizer.showMouse(1)


def init_services():
    """ Method to listen to events during the simulation

    Does nothing for the moment.
    """
    GameLogic.morse_services = MorseServices()

    try:
        GameLogic.morse_services.add_request_manager("morse.middleware.socket_request_manager.SocketRequestManager")
        GameLogic.morse_services.add_request_manager("morse.middleware.yarp_request_manager.YarpRequestManager")

        # The simulation 'supervision' always uses at least sockets for requests.
        GameLogic.morse_services.register_request_manager_mapping("simulation", "SocketRequestManager")

        # Services can be imported *only* after GameLogic.morse_services
        # has been created. Else @service won't know where to register the RPC
        # callbacks.
        import morse.core.supervision_services

        print("======= SERVICE MANAGERS INITIALIZED ========")
    except MorseServiceError as e:
        #...no request manager :-(
        print("WARNING: " + str(e))
        print("======= SERVICE MANAGERS INITIALIZATION FAILED ========")


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
        quitActuator = contr.actuators['Quit_sim']
        contr.activate(quitActuator)
        sys.exit(-1)

    if hasattr(GameLogic, "morse_services"):
        # let the service managers process their inputs/outputs
        GameLogic.morse_services.process()


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
    print ('######### TERMINATING INSTANCES... ########')
    # Force the deletion of the sensor objects
    for obj, component_instance in GameLogic.componentDict.items():
        del obj

    # Force the deletion of the robot objects
    for obj, robot_instance in GameLogic.robotDict.items():
        del obj

    print ('######### CLOSING REQUEST MANAGERS... ########')
    del GameLogic.morse_services

    print ('######### CLOSING MIDDLEWARES... ########')
    # Force the deletion of the middleware objects
    for obj, mw_instance in GameLogic.mwDict.items():
        if mw_instance:
            mw_instance.cleanup()
            #import gc
            #print ("At closing time, %s has %s references" % (mw_instance, gc.get_referents(mw_instance)))
            del obj


def finish(contr):
    """Close the open ports."""
    sensor = contr.sensors['ESC_KEY']

    #execute only when the ESC key is released (if we don't test that,
    #the code get executed two time, when pressed, and when released)
    if not sensor.positive and sensor.triggered:
        close_all(contr)

        quitActuator = contr.actuators['Quit_sim']
        contr.activate(quitActuator)

        print ('######### EXITING SIMULATION ########')


def restart(contr):
    """Close the open ports."""
    sensor = contr.sensors['F11_KEY']

    # Execute only when the F11 key is released (if we don't test that,
    #  the code get executed two times, when pressed, and when released)
    if not sensor.positive and sensor.triggered:

        print ("### Replacing everything ###")
        reset_objects(contr)
        return

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
