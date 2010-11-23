import sys
import re
import GameLogic
import Rasterizer

# Import this library to recover the Python version
import platform

# The file component_config.py is at the moment included
#  in the .blend file of the scene
import component_config



# Create a list of the robots in the scene
def create_dictionaries ():
    """Creation of a list of all the robots and components in the scene.
       Uses the properties of the objects to determine what they are."""

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

    scene = GameLogic.getCurrentScene()

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

    # Get the middlewares
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



def create_instance(obj, parent=None):
    """ Dynamically load a Python module and create an instance object
        of the class defined within. """
    # Read the path and class of the object from the Logic Properties
    source_file = obj['Path']
    module_name = re.sub('/', '.', source_file)
    print ("Path to Component Class: %s" % module_name)
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
        print ("No modifiers found in configuration file")
        return

    for component_name, mw_data in component_list.items():
        mw_name = mw_data[0]
        # Prefix the name of the component with 'OB'
        # Will only be necessary until the change to Blender 2.5
        if GameLogic.pythonVersion < 3:
            component_name = 'OB' + component_name

        print ("Component: '%s' using middleware '%s'" % (component_name, mw_name))
        found = False
        # Look for the listed mw in the dictionary of active mw's
        for mw_obj, mw_instance in GameLogic.mwDict.items():
            if mw_name in mw_obj.name:
                found = True
                # Get the instance of the object
                try:
                    instance = GameLogic.componentDict[component_name]
                except KeyError as detail:
                    print ("Component listed in component_config.py not found in scene: {0}".format(detail))
                    continue

                # Make the middleware object take note of the component
                mw_instance.register_component(component_name, instance, mw_data)

        if not found:
            print ("WARNING: There is no '%s' middleware object in the scene." % mw_name)

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
        return

    for component_name, mod_list in component_list.items():
        # Prefix the name of the component with 'OB'
        # Will only be necessary until the change to Blender 2.5
        if GameLogic.pythonVersion < 3:
            component_name = 'OB' + component_name

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
    """ Open the communication ports for administration."""
    print ('\n######## SCENE INITIALIZATION ########')
    # Get the version of Python used, according to the pythonpath
    # This is used to determine also the version of Blender
    python_version = platform.python_version()
    print ("Python Version: {0}".format(python_version))
    # Chop the version to only 3 chars: #.#  and convert to a number
    GameLogic.pythonVersion = float(python_version[:3])
    GameLogic.morse_initialised = False
    init_ok = True

    print ('======== COMPONENT DICTIONARY INITIALISATION =======')
    init_ok = create_dictionaries()
    init_ok = init_ok and add_modifiers()
    init_ok = init_ok and link_middlewares()

    if init_ok:
        print ('======= COMPONENT DICTIONARY INITIALISED =======')
        check_dictionaries()
        GameLogic.morse_initialised = True
    else:
        print ('======= INITIALISATION FAILED!!! =======')
        print ("Exiting the simulation!")
        contr = GameLogic.getCurrentController()
        close_all(contr)

    #Display the mouse in the simulator
    #Rasterizer.showMouse(1)


def admin(contr):
    """ Method to listen to events during the simulation

    Does nothing for the moment.
    """
    return


def close_all(contr):
    print ('######### TERMINATING INSTANCES... ########')
    # Force the deletion of the sensor objects
    for obj, component_instance in GameLogic.componentDict.items():
        del obj

    # Force the deletion of the robot objects
    for obj, robot_instance in GameLogic.robotDict.items():
        del obj

    # Force the deletion of the middleware objects
    for obj, mw_instance in GameLogic.mwDict.items():
        if mw_instance:
            mw_instance.cleanup()
            #import gc
            #print ("At closing time, %s has %s references" % (mw_instance, gc.get_referents(mw_instance)))
            del obj

    quitActuator = contr.actuators['Quit_sim']
    contr.activate(quitActuator)

    print ('######### EXITING SIMULATION ########')



def finish(contr):
    """Close the open ports."""
    sensor = contr.sensors['ESC_KEY']

    #execute only when the ESC key is released (if we don't test that,
    #the code get executed two time, when pressed, and when released)
    if not sensor.positive and sensor.triggered:
        close_all(contr)


def restart(contr):
    """Close the open ports."""
    sensor = contr.sensors['F11_KEY']

    #execute only when the F11 key is released (if we don't test that,
    #the code get executed two times, when pressed, and when released)
    if not sensor.positive and sensor.triggered:
        print ('######### CLOSING PORTS... ########')

        # TODO: Reimplement the restart function,
        #  by killing the objects created, and then
        #  calling the init function again.

        # ALL THE FOLLOWING DOES NOT WORK

        for obj, component_instance in GameLogic.componentDict.items():
            del obj

        for obj, robot_instance in GameLogic.robotDict.items():
            del obj

        # Force the deletion of the middleware objects
        for obj, mw_instance in GameLogic.mwDict.items():
            del obj

        init(contr)

        print ('######### RESTARTING SIMULATION ########')
