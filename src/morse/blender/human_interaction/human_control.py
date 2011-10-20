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

from bge import logic, events, render
import math
from mathutils import Matrix

AZERTY = False

# use different keys for QWERTZ/QWERTY or AZERTY keyboards
RIGHT  = events.DKEY
TURN_RIGHT = events.EKEY
BACKWARDS = events.SKEY

if AZERTY:
    FORWARDS = events.ZKEY
    LEFT = events.QKEY
    TURN_LEFT = events.AKEY
else:
    FORWARDS = events.WKEY
    LEFT = events.AKEY
    TURN_LEFT = events.QKEY

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
        # key[0] == events.keycode, key[1] = status
        if key[1] == logic.KX_INPUT_ACTIVE:
            if key[0] == FORWARDS:
                move_speed[0] = speed
            elif key[0] == BACKWARDS:
                move_speed[0] = -speed
            elif key[0] == TURN_LEFT:
                rotation_speed[2] = speed
            elif key[0] == TURN_RIGHT:
                rotation_speed[2] = -speed
            elif key[0] == RIGHT:
                move_speed[1] = -speed
            elif key[0] == LEFT:
                move_speed[1] = speed

            # The second parameter of 'applyMovement' determines
            #  a movement with respect to the object's local
            #  coordinate system
            human.applyMovement( move_speed, True )
            human.applyRotation( rotation_speed, True )

            """
            if key[0] == events.UPARROWKEY:
                move_speed[0] = speed
            elif key[0] == events.DOWNARROWKEY:
                move_speed[0] = -speed
            elif key[0] == events.LEFTARROWKEY:
                rotation_speed[2] = speed
            elif key[0] == events.RIGHTARROWKEY:
                rotation_speed[2] = -speed
            elif key[0] == events.AKEY:
                move_speed[2] = speed
            elif key[0] == events.EKEY:
                move_speed[2] = -speed
            """

        elif key[1] == logic.KX_INPUT_JUST_ACTIVATED:
            # Other actions activated with the keyboard
            # Reset camera to center
            if key[0] == events.NKEY and keyboard.positive:
                reset_view(contr)
            # Switch between look and manipulate
            elif key[0] == events.XKEY:
                toggle_manipulate(contr)

def read_status(contr):
    """ Check if the human is moving and set the flags
    
    This will trigger the walking animation even when the human
    is controlled via a motion actuator
    """
    human = contr.owner
    scene = logic.getCurrentScene()
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


def set_human_animation(contr):
    """ Toggle the animation actions (walking, standing still...) of 
    the armature. 
    """
    # Get sensor named Mouse
    armature = contr.owner
    keyboard = contr.sensors['All_Keys']

    keylist = keyboard.events
    pressed = []      #all keys that are currently pressed
    for key in keylist:
        # key[0] == events.keycode, key[1] = status
        if key[1] == logic.KX_INPUT_JUST_ACTIVATED:
            pressed.append(key[0])
            # Keys for moving forward or turning
            """
            if key[0] == events.WKEY or key[0] == events.ZKEY:
                armature['movingForward'] = True
            elif key[0] == events.SKEY:
                armature['movingBackward'] = True
            """
            # TEST: Read the rotation of the bones in the armature
            if key[0] == events.BKEY:
                read_pose(contr)
            #elif key[0] == events.VKEY:
                #reset_pose(contr)
        #elif key[1] == logic.KX_INPUT_JUST_RELEASED:
            """            
            if key[0] == events.WKEY or key[0] == events.ZKEY:
                armature['movingForward'] = False
            elif key[0] == events.SKEY:
                armature['movingBackward'] = False
        """
        elif key[1] == logic.KX_INPUT_ACTIVE:
            pressed.append(key[0])
    
    if (FORWARDS in pressed or LEFT in pressed or BACKWARDS in pressed or
        RIGHT in pressed):
        armature['movingForward'] = True
    else:
        armature['movingForward'] = False


def head_control(contr):
    """ Move the target of the head and camera

    Use the movement of the mouse to determine the rotation
    for the human head and camera. """
    # get the object this script is attached to
    human = contr.owner
    scene = logic.getCurrentScene()
    target = scene.objects['Target_Empty']
    POS_EMPTY = scene.objects['POS_EMPTY']
    Head_Empty = scene.objects['Head_Empty']
    right_hand = scene.objects['Hand_Grab.R']
    mmb = contr.sensors['MMB']


    # If the manipulation mode is active, an object is grabbed
    # and the Middle Mouse Button is pressed, do nothing
    if (human['Manipulate'] and right_hand['selected'] != 'None' and
        right_hand['selected'] != '' and mmb.positive):
        return

    if mmb.positive:
        target = scene.objects['IK_Target_Empty.R']

    # Get sensor named Mouse
    mouse = contr.sensors['Mouse']

    if mouse.positive:
        # get width and height of game window
        width = render.getWindowWidth()
        height = render.getWindowHeight()

        # get mouse movement from function
        move = mouse_move(human, mouse, width, height)

        # set mouse sensitivity
        sensitivity = human['Sensitivity']

        # Amount, direction and sensitivity
        left_right = move[0] * sensitivity
        up_down = move[1] * sensitivity

        if not human['FOCUSED']:
            POS_EMPTY.applyRotation([0.0, 0.0, left_right], True)
            if not ((Head_Empty.localOrientation.to_euler()[1] >= 0.7
                     and up_down < 0) or
                    (Head_Empty.localOrientation.to_euler()[1] <= -0.4
                     and up_down > 0)) and not human['Manipulate']:
                # capping the rotation to prevent the camera to be upside down
                if not mmb.positive:
                    Head_Empty.applyRotation([0.0, -up_down, 0.0], True)
                target.applyMovement([0.0, 0.0, up_down], True)
            elif human['Manipulate']:
                Head_Empty.applyRotation([0.0, -up_down, 0.0], True)
                target.applyMovement([0.0, 0.0, up_down], True)

        # Reset mouse position to the centre of the screen
        # Using the '//' operator (floor division) to produce an integer result
        render.setMousePosition(width//2, height//2)


def hand_control(contr):
    """ Move the hand following the mouse

    Use the movement of the mouse to determine the rotation
    for the IK arm (right arm)
    
    stays for better placing of objects - >(QKEY + EKEY) to rotate body<
    """
    # get the object this script is attached to
    human = contr.owner
    scene = logic.getCurrentScene()
    target = scene.objects['IK_Target_Empty.R']
    right_hand = scene.objects['Hand_Grab.R']
    mmb = human.sensors['MMB']

    # If the manipulation mode is inactive, do nothing
    if not human['Manipulate']:
        return

    # set mouse sensitivity
    sensitivity = human['Sensitivity']

    # Get sensors for mouse wheel
    wheel_up = contr.sensors['Wheel_Up']
    wheel_down = contr.sensors['Wheel_Down']

    if wheel_up.positive:
        front = 50.0 * sensitivity
        target.applyMovement([front, 0.0, 0.0], True)

    if wheel_down.positive:
        back = -50.0 * sensitivity
        target.applyMovement([back, 0.0, 0.0], True)

    # If nothing grabbed or Middle Mouse Button is not pressed,
    # do nothing of the following
    if (right_hand['selected'] == 'None' or right_hand['selected'] == '' or
        (not mmb.positive)):
        #use head_control for this
        return

    # Get sensor named Mouse
    mouse = contr.sensors['Mouse']

    if mouse.positive:
        # get width and height of game window
        width = render.getWindowWidth()
        height = render.getWindowHeight()

        # get mouse movement from function
        move = mouse_move(human, mouse, width, height)

        # Amount, direction and sensitivity
        left_right = move[0] * sensitivity
        up_down = move[1] * sensitivity

        if not human['FOCUSED']:
            target.applyMovement([0.0, left_right, 0.0], True)
            target.applyMovement([0.0, 0.0, up_down], True)

        # Reset mouse position to the centre of the screen
        # Using the '//' operator (floor division) to produce an integer result
        render.setMousePosition(width//2, height//2)


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
    scene = logic.getCurrentScene()
    target = scene.objects['Target_Empty']
    # Reset the Empty object to its original position
    target.localPosition = [1.3, 0.0, 1.7]


def toggle_manipulate(contr):
    """ Switch mouse control between look and manipulate """
    human = contr.owner
    scene = logic.getCurrentScene()
    hand_target = scene.objects['IK_Target_Empty.R']
    head_target = scene.objects['Target_Empty']
    right_hand = scene.objects['Hand_Grab.R']

    if human['Manipulate']:
        #render.showMouse(False)
        human['Manipulate'] = False
        # Place the hand beside the body
        if right_hand['selected'] == 'None' or right_hand['selected'] == '':
            hand_target.localPosition = [0.3, -0.3, 0.9]
            head_target.setParent(human)
            head_target.localPosition = [1.3, 0.0, 1.7]
    else:
        #render.showMouse(True)
        human['Manipulate'] = True
        head_target.setParent(hand_target)
        # Place the hand in a nice position
        hand_target.localPosition = [0.6, 0.0, 1.4]
        head_target.worldPosition = hand_target.worldPosition	



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
    hipsdown = contr.actuators["SitDown"]
    hipsup = contr.actuators["StandUp"]

    # Sitdown
    if sit_down_key.positive and human['statusStandUp']:
        contr.activate(sitdown)
        contr.activate(hipsdown)
        human['statusStandUp'] = False

    # Standup
    elif sit_down_key.positive and not human['statusStandUp']:
        contr.activate(standup)
        contr.activate(hipsup)
        human['statusStandUp'] = True



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

def applyrotate(destOr, owner):
    """
    Rotates the owner of this script to the given orientation.
    'smoothness' defines the speed of this rotation
    """
    smoothness = 10
    currOr = owner.worldOrientation
    dZ = [0.0,0.0,0.0]

    dZ = currOr.to_euler()[2] - destOr.to_euler()[2]

    # Blender allows multiples of 360 deg and negative angles
    # this is to get rid of those
    while(dZ < math.pi):
        dZ = dZ + 2 * math.pi
    while(dZ > math.pi):
        dZ = dZ - 2 * math.pi

    owner.worldOrientation = (owner.worldOrientation *
                              Matrix.Rotation(-dZ/(smoothness), 3, 'Z'))
    # turn around a bit

def rotate(co):
    """
    Set the human orientation in reference to the camera orientation.
    """
    ow = co.owner
    keyboard = co.sensors['Keyboard']
    pos =  logic.getCurrentScene().objects['POS_EMPTY']

    keylist = keyboard.events

    k = []    #initiate a list with all currently pressed keys
    for key in keylist:
        if key[1] ==  logic.KX_INPUT_ACTIVE:
            k.append(key[0])        # add all pressed keys to a list - as ASCII CODES

    ow.worldPosition = pos.worldPosition

    if pos['Manipulate']:
        ow.worldOrientation = pos.worldOrientation
        # lock camera to head in Manipulation Mode
    else:
        if FORWARDS in k and not(LEFT in k or RIGHT in k):  
            applyrotate(pos.worldOrientation, ow)
        elif LEFT in k and not(FORWARDS in k or BACKWARDS in k):
            applyrotate(pos.worldOrientation *
                        Matrix.Rotation(math.pi / 2, 3, 'Z'), ow)
            # turn around 90 deg
        elif RIGHT in k and not(FORWARDS in k or BACKWARDS in k):
            applyrotate(pos.worldOrientation *
                        Matrix.Rotation(math.pi * 3/2, 3, 'Z'), ow)
            # turn around 270 deg
        elif LEFT in k and FORWARDS in k:
            applyrotate(pos.worldOrientation *
                        Matrix.Rotation(math.pi / 4, 3, 'Z'), ow)
            # turn around 45 deg
        elif RIGHT in k and FORWARDS in k:
            applyrotate(pos.worldOrientation *
                        Matrix.Rotation(math.pi * 7 / 4, 3, 'Z'), ow)
            # turn around 315 deg
        elif BACKWARDS in k and not(LEFT in k or RIGHT in k):
            applyrotate(pos.worldOrientation *
                        Matrix.Rotation(math.pi, 3, 'Z'), ow)
            # turn around 180 deg
        elif LEFT in k and BACKWARDS in k:
            applyrotate(pos.worldOrientation *
                        Matrix.Rotation(math.pi * 3/4, 3, 'Z'), ow)
            # turn around 135 deg
        elif RIGHT in k and BACKWARDS in k:
            applyrotate(pos.worldOrientation *
                        Matrix.Rotation(math.pi * 5/4, 3, 'Z'), ow)
            # turn around 225 deg


