import logging; logger = logging.getLogger("morse." + __name__)

######################################################
#
#    human_control.py        Blender 2.55
#
#    Modified version of
#      view_camera.py by Gilberto Echeverria
#
#    Gilberto Echeverria
#    26 / 12 / 2010
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
            # Also add the key corresponding key for an AZERTY keyboard
            if key[0] == bge.events.WKEY or key[0] == bge.events.ZKEY:
                move_speed[0] = speed
            elif key[0] == bge.events.SKEY:
                move_speed[0] = -speed
            # Also add the key corresponding key for an AZERTY keyboard
            elif key[0] == bge.events.AKEY or key[0] == bge.events.QKEY:
                rotation_speed[2] = speed
            elif key[0] == bge.events.DKEY:
                rotation_speed[2] = -speed
            elif key[0] == bge.events.RKEY:
                move_speed[1] = speed
            elif key[0] == bge.events.FKEY:
                move_speed[1] = -speed

            # The second parameter of 'applyMovement' determines
            #  a movement with respect to the object's local
            #  coordinate system
            human.applyMovement( move_speed, True )
            human.applyRotation( rotation_speed, True )

            """
            if key[0] == bge.events.UPARROWKEY:
                move_speed[0] = speed
            elif key[0] == bge.events.DOWNARROWKEY:
                move_speed[0] = -speed
            elif key[0] == bge.events.LEFTARROWKEY:
                rotation_speed[2] = speed
            elif key[0] == bge.events.RIGHTARROWKEY:
                rotation_speed[2] = -speed
            elif key[0] == bge.events.AKEY:
                move_speed[2] = speed
            elif key[0] == bge.events.EKEY:
                move_speed[2] = -speed
            """

        elif key[1] == bge.logic.KX_INPUT_JUST_ACTIVATED:
            # Other actions activated with the keyboard
            # Reset camera to center
            if key[0] == bge.events.NKEY and keyboard.positive:
                reset_view(contr)
            # Switch between look and manipulate
            elif key[0] == bge.events.XKEY:
                toggle_manipulate(contr)

def read_status(contr):
    """ Check if the human is moving and set the flags
    
    This will trigger the walking animation even when the human
    is controlled via a motion actuator
    """
    human = contr.owner
    scene = bge.logic.getCurrentScene()
    armature = scene.objects['HumanArmature']
    tolerance = 0.001

    # TODO: Do not change the movement properties until the controllers
    #  are properly implemented to use velocity commands
    if False:
        speed = human.getLinearVelocity()
        logger.debug("Man going at speed [%.4f, %.4f, %.4f]" % (speed[0], speed[1], speed[2]))
        if speed[0] > tolerance:
           armature['movingForward'] = True 
        elif speed[0] < -tolerance:
           armature['movingBackward'] = True 
        else:
           armature['movingForward'] = False 
           armature['movingBackward'] = False 


def human_actions(contr):
    """ Toggle the animation actions of the armature """
    # Get sensor named Mouse
    armature = contr.owner
    keyboard = contr.sensors['All_Keys']

    keylist = keyboard.events
    for key in keylist:
        # key[0] == bge.events.keycode, key[1] = status
        if key[1] == bge.logic.KX_INPUT_JUST_ACTIVATED:
            # Keys for moving forward or turning
            if key[0] == bge.events.WKEY or key[0] == bge.events.ZKEY:
                armature['movingForward'] = True
            elif key[0] == bge.events.SKEY:
                armature['movingBackward'] = True

            # TEST: Read the rotation of the bones in the armature
            elif key[0] == bge.events.BKEY:
                read_pose(contr)
            #elif key[0] == bge.events.VKEY:
                #reset_pose(contr)
        elif key[1] == bge.logic.KX_INPUT_JUST_RELEASED:
            if key[0] == bge.events.WKEY or key[0] == bge.events.ZKEY:
                armature['movingForward'] = False
            elif key[0] == bge.events.SKEY:
                armature['movingBackward'] = False


def head_control(contr):
    """ Move the target of the head and camera

    Use the movement of the mouse to determine the rotation
    for the human head and camera. """
    # get the object this script is attached to
    human = contr.owner
    scene = bge.logic.getCurrentScene()
    target = scene.objects['Target_Empty']

    # set mouse sensitivity
    sensitivity = human['Sensitivity']

    # If the manipulation mode is active, do nothing
    if human['Manipulate']:
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


def hand_control(contr):
    """ Move the hand following the mouse

    Use the movement of the mouse to determine the rotation
    for the IK arm (right arm) """
    # get the object this script is attached to
    human = contr.owner
    scene = bge.logic.getCurrentScene()
    target = scene.objects['IK_Target_Empty.R']

    # set mouse sensitivity
    sensitivity = human['Sensitivity']

    # If the manipulation mode is inactive, do nothing
    if not human['Manipulate']:
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

    # Get sensors for mouse wheel
    wheel_up = contr.sensors['Wheel_Up']
    wheel_down = contr.sensors['Wheel_Down']

    if wheel_up.positive:
        front = 50.0 * sensitivity
        target.applyMovement([front, 0.0, 0.0], True)

    if wheel_down.positive:
        back = -50.0 * sensitivity
        target.applyMovement([back, 0.0, 0.0], True)


def read_pose(contr):
    armature = contr.owner
    logger.info("The armature is: '%s' (%s)" % (armature, type(armature)))

    for channel in armature.channels:
        if 'X_' not in channel.name:
            rotation = channel.joint_rotation
            logger.info("\tChannel '%s': (%.4f, %.4f, %.4f)" % (channel, rotation[0], rotation[1], rotation[2]))


def reset_pose(contr):
    armature = contr.owner
    logger.info("Trying to reset the posture:")
    for channel in armature.channels:     
        channel.rotation_mode = 6
        
        channel.joint_rotation = [0.0, 0.0, 0.0]

        rotation = channel.joint_rotation
        logger.info("\tChannel '%s': (%.4f, %.4f, %.4f)" % (channel, rotation[0], rotation[1], rotation[2]))

    armature.update()

def reset_view(contr):
    """ Make the human model look forward """
    human = contr.owner
    scene = bge.logic.getCurrentScene()
    target = scene.objects['Target_Empty']
    # Reset the Empty object to its original position
    target.localPosition = [1.3, 0.0, 1.7]


def toggle_manipulate(contr):
    """ Switch mouse control between look and manipulate """
    human = contr.owner
    scene = bge.logic.getCurrentScene()
    hand_target = scene.objects['IK_Target_Empty.R']
    head_target = scene.objects['Target_Empty']

    if human['Manipulate']:
        #bge.render.showMouse(False)
        human['Manipulate'] = False
        # Place the hand beside the body
        hand_target.localPosition = [0.3, -0.3, 0.9]
        head_target.setParent(human)
        head_target.localPosition = [0.5, 0.0, 1.6]
    else:
        #bge.render.showMouse(True)
        human['Manipulate'] = True
        head_target.setParent(hand_target)
        # Place the hand in a nice position
        hand_target.localPosition = [0.6, 0.0, 1.4]
        # Place the head in the same place
        head_target.localPosition = [0.0, 0.0, 0.0]


def toggle_sit(contr):
    """ Change the stance of the human model

    Make the human sit down or stand up, using a preset animation.
    """
    human = contr.owner

    # get the keyboard sensor
    sit_down_key = contr.sensors["sit_down"]

    # get the actuators
    sitdown = contr.actuators["sitdown"]
    standup = contr.actuators["standup"]

    # Sitdown
    if sit_down_key.positive and human['statusStandUp']:
        contr.activate(sitdown)
        human['statusStandUp'] = False

    # Standup
    elif sit_down_key.positive and not human['statusStandUp']:
        contr.activate(standup)
        human['statusStandUp'] = True


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
