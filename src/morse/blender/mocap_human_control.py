import logging; logger = logging.getLogger("morse." + __name__)

######################################################
#
#    mocap_human_control.py        Blender 2.61
#
#    Modified version of
#      human_control.py by Michael Karg
#
#    Gilberto Echeverria
#    09 / 07 / 2012
#
######################################################

import bge
import math

def move(contr):
    """ Read the keys for specific combinations
        that will make the camera move in 3D space. """
    # get the object this script is attached to
    human = contr.owner

    # set the movement speed
    speed = human['Speed']

    # Get sensor named Mouse
    keyboard = contr.sensors['All_Keys']

    # Default movement speed
    move_speed = [0.0, 0.0, 0.0]
    rotation_speed = [0.0, 0.0, 0.0]

    keylist = keyboard.events
    for key in keylist:
        # key[0] == bge.events.keycode, key[1] = status
        if key[1] == bge.logic.KX_INPUT_ACTIVE:
            if key[0] == bge.events.IKEY:
                move_speed[0] = speed
            elif key[0] == bge.events.KKEY:
                move_speed[0] = -speed
            elif key[0] == bge.events.JKEY:
                rotation_speed[2] = speed
            elif key[0] == bge.events.LKEY:
                rotation_speed[2] = -speed
            elif key[0] == bge.events.UKEY:
                move_speed[1] = speed
            elif key[0] == bge.events.OKEY:
                move_speed[1] = -speed

            # The second parameter of 'applyMovement' determines
            #  a movement with respect to the object's local
            #  coordinate system
            human.applyMovement( move_speed, True )
            human.applyRotation( rotation_speed, True )

        """
        elif key[1] == bge.logic.KX_INPUT_JUST_ACTIVATED:
            # Other actions activated with the keyboard
            # Reset camera to center
            if key[0] == bge.events.NKEY and keyboard.positive:
                reset_view(contr)
        """


def head_control(contr):
    """ Move the target of the head and camera

    Use the movement of the mouse to determine the rotation
    for the human head and camera. """
    # get the object this script is attached to
    human = contr.owner
    scene = bge.logic.getCurrentScene()
    target = scene.objects['Head_Empty']
    # get the camera on the human head
    camera = scene.objects['Human_Camera']

    # set mouse sensitivity
    sensitivity = human['Sensitivity']

    # Do not move the camera if the current view is using another camera
    if camera != bge.logic.getCurrentScene().active_camera:
        return

    # Get sensor named Mouse
    mouse = contr.sensors['Mouse']

    if mouse.positive:
        # get width and height of game window
        width = bge.render.getWindowWidth()
        height = bge.render.getWindowHeight()

        # get mouse movement from function
        move = mouse_move(human, mouse, width, height)

        # Amount, direction and sensitivity
        left_right = move[0] * sensitivity
        up_down = move[1] * sensitivity

        target.applyMovement([0.0, left_right, 0.0], True)
        target.applyMovement([0.0, 0.0, up_down], True)

        # Reset mouse position to the centre of the screen
        # Using the '//' operator (floor division) to produce an integer result
        bge.render.setMousePosition(width//2, height//2)


def read_pose(contr):
    """ Test function to access the bone rotation data """
    armature = contr.owner
    logger.info("The armature is: '%s' (%s)" % (armature, type(armature)))

    for channel in armature.channels:
        if 'X_' not in channel.name:
            rotation = channel.joint_rotation
            logger.info("\tChannel '%s': (%.4f, %.4f, %.4f)" % (channel, rotation[0], rotation[1], rotation[2]))


def reset_view(contr):
    """ Make the human model look forward """
    human = contr.owner
    scene = bge.logic.getCurrentScene()
    target = scene.objects['Head_Empty']
    # Reset the Empty object to its original position
    target.localPosition = [0.5, 0.0, 1.6]


def near_object(contr):
    """ Store the object that is near the hand
    
    This script is called from the logic bricks of Hand_Grab.R
    """
    scene = bge.logic.getCurrentScene()
    hand_empty = scene.objects['Hand_Grab.R']
    near_sensor = hand_empty.sensors['Near']

    near_object = near_sensor.hitObject
    hand_empty['Near_Object'] = near_object

    #if near_object != None:
        #hand_empty.parent.localOrientation = [math.pi/2, 0.0, 0.0]
        #logger.debug(near_object.name + " can be grasped!")


def grabbing(contr):
    """ Mark an object as selected by the user """
    scene = bge.logic.getCurrentScene()
    human = contr.owner
    hand_empty = scene.objects['Hand_Grab.R']
    #sphere = scene.objects['SelectionSphere']
    lmb = human.sensors['LMB']
    selected_object = hand_empty['Near_Object']

    # Check that a button was pressed
    if lmb.getButtonStatus(bge.events.LEFTMOUSE) == bge.logic.KX_INPUT_JUST_ACTIVATED:
        # Check that no other object is being carried
        if contr.owner['DraggedObject'] == None or contr.owner['DraggedObject'] == '':
            # If the object is draggable
            if selected_object != None and selected_object != '':
                # Clear the previously selected object, if any
                contr.owner['DraggedObject'] = selected_object
                # Remove Physic simulation
                #selected_object.suspendDynamics()
                # Parent the selected object to the hand target
                selected_object.setParent (hand_empty)

    # Drop the object when the left mouse button is released
    if lmb.getButtonStatus(bge.events.LEFTMOUSE) == bge.logic.KX_INPUT_JUST_RELEASED:
    #if lmb.getButtonStatus(bge.events.RIGHTMOUSE) == bge.logic.KX_INPUT_JUST_ACTIVATED:
        # Clear the previously selected object, if any
        if contr.owner['DraggedObject'] != None and contr.owner['DraggedObject'] != '':
            previous_object = contr.owner["DraggedObject"]
            # Remove the parent
            previous_object.removeParent()
            # Place the object on the nearest surface
            #morse.helpers.place_object.do_place(previous_object)
            # Reset rotation of object
            previous_object.worldOrientation = [0.0, 0.0, 0.0]
            # Restore Physics simulation
            #previous_object.restoreDynamics()
            #previous_object.setLinearVelocity([0, 0, 0])
            #previous_object.setAngularVelocity([0, 0, 0])
            # Clear the object from dragged status
            contr.owner['DraggedObject'] = None



def mouse_move(human, mouse, width, height):
    """ Get the movement of the mouse as an X, Y coordinate. """
    # distance moved from screen center
    # Using the '//' operator (floor division) to produce an integer result
    x = width//2 - mouse.position[0]
    y = height//2 - mouse.position[1]

    # intialize mouse so it doesn't jerk first time
    try:
        human['mouseInit']
    except KeyError:
        x = 0
        y = 0
        # bug in Add Property
        # can't use True.  Have to use 1
        human['mouseInit'] = 1

    # return mouse movement
    return (x, y)
