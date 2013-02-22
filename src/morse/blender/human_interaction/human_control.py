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

from morse.core import blenderapi
import math
from mathutils import Matrix

AZERTY = False

# use different keys for QWERTZ/QWERTY or AZERTY keyboards
RIGHT  = blenderapi.DKEY
TURN_RIGHT = blenderapi.EKEY
BACKWARDS = blenderapi.SKEY

if AZERTY:
    FORWARDS = blenderapi.ZKEY
    LEFT = blenderapi.QKEY
    TURN_LEFT = blenderapi.AKEY
else:
    FORWARDS = blenderapi.WKEY
    LEFT = blenderapi.AKEY
    TURN_LEFT = blenderapi.QKEY

def lock_movement(contr):
    human = contr.owner
    keyboard = contr.sensors['All_Keys']

    scene = blenderapi.scene()

    if scene.active_camera.name != 'CameraFP':
        return

    keylist = keyboard.events
    for key in keylist:
        # key[0] == blenderapi.keycode, key[1] = status
        if key[1] == blenderapi.input_just_activated():
            if key[0] == blenderapi.F5KEY:
                human['move_cameraFP'] = not human['move_cameraFP']
                if human['move_cameraFP']:
                    logger.info("Moving CameraFP")
                else:
                    logger.info("Moving Human")
                

def move(contr):
    """ Read the keys for specific combinations
        that will make the camera move in 3D space. """
    
    # Get the currently active camera to adapt control method
    scene = blenderapi.scene()
    active_camera = scene.active_camera
    
    # get the object this script is attached to
    human = contr.owner

    # if the human is external, do nothing
    if human.get('External_Robot_Tag') or human['disable_keyboard_control']:
        return
    
    # get the suffix of the human to reference the right objects
    suffix = human.name[-4:] if human.name[-4] == "." else ""

    # set the movement speed
    speed = human['Speed']

    # Get sensor named Mouse
    keyboard = contr.sensors['All_Keys']

    # Default movement speed
    move_speed = [0.0, 0.0, 0.0]
    rotation_speed = [0.0, 0.0, 0.0]

    human_camera = "Human_Camera" + suffix

    if human['move_cameraFP'] and active_camera.name != human_camera:
        return

    keylist = keyboard.events
    for key in keylist:
        # key[0] == blenderapi.keycode, key[1] = status
        if key[1] == blenderapi.input_active():
            if human['Manipulate']:
                if key[0] == FORWARDS:
                    move_speed[0] = speed
                elif key[0] == BACKWARDS:
                    move_speed[0] = -speed
                elif key[0] == TURN_LEFT:
                    rotation_speed[2] = speed
                elif key[0] == TURN_RIGHT:
                    rotation_speed[2] = -speed
                elif key[0] == RIGHT:
                    if active_camera.name == human_camera:
                        move_speed[1] = -speed
                    else:
                        rotation_speed[2] = -speed
                elif key[0] == LEFT:
                    if active_camera.name == human_camera:
                        move_speed[1] = speed
                    else:
                        rotation_speed[2] = speed
            else:
                if key[0] in (FORWARDS, BACKWARDS, LEFT, RIGHT):
                    move_speed[0] = speed
                    if active_camera.name != human_camera and key[0] == BACKWARDS:
                        move_speed[0] = -speed

            # The second parameter of 'applyMovement' determines
            #  a movement with respect to the object's local
            #  coordinate system
            human.applyMovement( move_speed, True )
            human.applyRotation( rotation_speed, True )

            """
            if key[0] == blenderapi.UPARROWKEY:
                move_speed[0] = speed
            elif key[0] == blenderapi.DOWNARROWKEY:
                move_speed[0] = -speed
            elif key[0] == blenderapi.LEFTARROWKEY:
                rotation_speed[2] = speed
            elif key[0] == blenderapi.RIGHTARROWKEY:
                rotation_speed[2] = -speed
            elif key[0] == blenderapi.AKEY:
                move_speed[2] = speed
            elif key[0] == blenderapi.EKEY:
                move_speed[2] = -speed
            """

        elif key[1] == blenderapi.input_just_activated():
            # Other actions activated with the keyboard
            # Reset camera to center
            if key[0] == blenderapi.NKEY and keyboard.positive:
                reset_view(contr)
            # Switch between look and manipulate
            elif key[0] == blenderapi.XKEY:
                toggle_manipulate(contr)

def read_status(contr):
    """ Check if the human is moving and set the flags
    
    This will trigger the walking animation even when the human
    is controlled via a motion actuator
    """
    human = contr.owner
    scene = blenderapi.scene()
    
    # get the suffix of the human to reference the right objects
    suffix = human.name[-4:] if human.name[-4] == "." else ""
    
    armature = scene.objects['HumanArmature' + suffix]
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

    # get the suffix of the human to reference the right objects
    suffix = armature.name[-4:] if armature.name[-4] == "." else ""

    scene = blenderapi.scene()
    active_camera = scene.active_camera
    human = scene.objects[armature.parent.name + suffix]

    # if the human is external, do nothing
    if human.get('External_Robot_Tag') or human['disable_keyboard_control']:
        return    

    keyboard = contr.sensors['All_Keys']

    keylist = keyboard.events
    pressed = []      #all keys that are currently pressed
    for key in keylist:
        # key[0] == blenderapi.keycode, key[1] = status
        if key[1] == blenderapi.input_just_activated():
            pressed.append(key[0])
            # Keys for moving forward or turning
            """
            if key[0] == blenderapi.WKEY or key[0] == blenderapi.ZKEY:
                armature['movingForward'] = True
            elif key[0] == blenderapi.SKEY:
                armature['movingBackward'] = True
            """
            # TEST: Read the rotation of the bones in the armature
            if key[0] == blenderapi.BKEY:
                read_pose(contr)
            #elif key[0] == blenderapi.VKEY:
                #reset_pose(contr)
        #elif key[1] == blenderapi.input_just_released():
            """            
            if key[0] == blenderapi.WKEY or key[0] == blenderapi.ZKEY:
                armature['movingForward'] = False
            elif key[0] == blenderapi.SKEY:
                armature['movingBackward'] = False
        """
        elif key[1] == blenderapi.input_active():
            pressed.append(key[0])

    
    if human['move_cameraFP'] and active_camera.name != 'Human_Camera':
        return
    
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
    
    # if the human is external, do nothing
    if human.get('External_Robot_Tag') or human['disable_keyboard_control']:
        return

    # get the suffix of the human to reference the right objects
    suffix = human.name[-4:] if human.name[-4] == "." else ""

    scene = blenderapi.scene()
    target = scene.objects['Target_Empty' + suffix]
    POS_EMPTY = scene.objects['POS_EMPTY' + suffix]
    Head_Empty = scene.objects['Head_Empty' + suffix]
    right_hand = scene.objects['Hand_Grab.R' + suffix]
    camera = scene.objects['Human_Camera' + suffix]
    mmb = contr.sensors['MMB']

    # Do not move the camera if the current view is using another camera
    if camera != blenderapi.scene().active_camera:
        return

    # If the manipulation mode is active, an object is grabbed
    # and the Middle Mouse Button is pressed, do nothing
    if (human['Manipulate'] and right_hand['selected'] != 'None' and
        right_hand['selected'] != '' and mmb.positive):
        return

    if mmb.positive:
        target = scene.objects['IK_Target_Empty.R' + suffix]

    # Get sensor named Mouse
    mouse = contr.sensors['Mouse']

    if mouse.positive:
        # get width and height of game window
        width = blenderapi.render().getWindowWidth()
        height = blenderapi.render().getWindowHeight()

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
        blenderapi.render().setMousePosition(width//2, height//2)


def hand_control(contr):
    """ Move the hand following the mouse

    Use the movement of the mouse to determine the rotation
    for the IK arm (right arm)
    
    stays for better placing of objects - >(QKEY + EKEY) to rotate body<
    """
    # get the object this script is attached to
    human = contr.owner
    
    # if the human is external, do nothing
    if human.get('External_Robot_Tag') or human['disable_keyboard_control']:
        return

    # get the suffix of the human to reference the right objects
    suffix = human.name[-4:] if human.name[-4] == "." else ""
    
    scene = blenderapi.scene()
    target = scene.objects['IK_Target_Empty.R' + suffix]
    right_hand = scene.objects['Hand_Grab.R' + suffix]
    mmb = human.sensors['MMB']

    # If the manipulation mode is inactive, do nothing
    if not human['Manipulate']:
        return

    # set mouse sensitivity
    sensitivity = human['Sensitivity']

    # Get sensors for mouse wheel
    wheel_up = contr.sensors['Wheel_Up']
    wheel_down = contr.sensors['Wheel_Down']
    keyboard = contr.sensors['All_Keys']

    keylist = keyboard.events
    for key in keylist:
        if key[1] == blenderapi.input_none() and key[0] == blenderapi.LEFTCTRLKEY:
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
        width = blenderapi.render().getWindowWidth()
        height = blenderapi.render().getWindowHeight()

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
        blenderapi.render().setMousePosition(width//2, height//2)


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
    
    # if the human is external, do nothing
    if human.get('External_Robot_Tag') or human['disable_keyboard_control']:
        return
    
    # get the suffix of the human to reference the right objects
    suffix = human.name[-4:] if human.name[-4] == "." else ""
    
    scene = blenderapi.scene()
    target = scene.objects['Target_Empty' + suffix]
    # Reset the Empty object to its original position
    target.localPosition = [1.3, 0.0, 1.7]


def toggle_manipulate(contr):
    """ Switch mouse control between look and manipulate """
    human = contr.owner
    
    # if the human is external, do nothing
    if human.get('External_Robot_Tag') or human['disable_keyboard_control']:
        return

    # get the suffix of the human to reference the right objects
    suffix = human.name[-4:] if human.name[-4] == "." else ""
    
    scene = blenderapi.scene()
    hand_target = scene.objects['IK_Target_Empty.R' + suffix]
    head_target = scene.objects['Target_Empty' + suffix]
    right_hand = scene.objects['Hand_Grab.R' + suffix]

    if human['Manipulate']:
        #blenderapi.render().showMouse(False)
        human['Manipulate'] = False
        # Place the hand beside the body
        if right_hand['selected'] == 'None' or right_hand['selected'] == '' or right_hand['selected'] == None:
            hand_target.localPosition = [0.3, -0.3, 0.9]
            head_target.setParent(human)
            head_target.localPosition = [1.3, 0.0, 1.7]
    else:
        #blenderapi.render().showMouse(True)
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
        
    # if the human is external, do nothing
    if human.parent.get('External_Robot_Tag') or human.parent['disable_keyboard_control']:
        return

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

    # get the suffix of the human to reference the right objects
    suffix = ow.name[-4:] if ow.name[-4] == "." else ""
    
    keyboard = co.sensors['All_Keys']
    scene = blenderapi.scene()
    human_pos = co.owner
    pos =  scene.objects['POS_EMPTY' + suffix]
    active_camera = scene.active_camera
        
    # if the human is external, do nothing
    if human_pos.get('External_Robot_Tag') or human_pos['disable_keyboard_control']:
        return
    
    if human_pos['move_cameraFP'] and active_camera.name != ('Human_Camera'+suffix):
        return
    
    keylist = keyboard.events

    k = []    #initiate a list with all currently pressed keys
    for key in keylist:
        if key[1] ==  blenderapi.input_active():
            k.append(key[0])        # add all pressed keys to a list - as ASCII CODES

    pos.worldPosition = ow.worldPosition

    # Get active camera
    scene = blenderapi.scene()
    active_camera = scene.active_camera
    
    if ow['Manipulate']:
        ow.worldOrientation = pos.worldOrientation
        # lock camera to head in Manipulation Mode
    else:
        if FORWARDS in k and not(LEFT in k or RIGHT in k):  
            if active_camera.name == ("Human_Camera"+suffix):
                applyrotate(pos.worldOrientation, ow)
            else:
                applyrotate(human_pos.worldOrientation, ow) 
        elif LEFT in k and not(FORWARDS in k or BACKWARDS in k):
            if active_camera.name == ("Human_Camera"+suffix):
                applyrotate(pos.worldOrientation *
                            Matrix.Rotation(math.pi / 2, 3, 'Z'), ow)
            else: 
                applyrotate(human_pos.worldOrientation *
                            Matrix.Rotation(math.pi / 2, 3, 'Z'), ow)
            # turn around 90 deg
        elif RIGHT in k and not(FORWARDS in k or BACKWARDS in k):
            if active_camera.name == ("Human_Camera"+suffix):
                applyrotate(pos.worldOrientation *
                            Matrix.Rotation(math.pi * 3/2, 3, 'Z'), ow)
            else:
                applyrotate(human_pos.worldOrientation * Matrix.Rotation(math.pi * 3/2, 3, 'Z'), ow)
            # turn around 270 deg
        elif LEFT in k and FORWARDS in k:
            if active_camera.name == ("Human_Camera"+suffix):
                applyrotate(pos.worldOrientation *
                            Matrix.Rotation(math.pi / 4, 3, 'Z'), ow)
            else: 
                applyrotate(human_pos.worldOrientation *
                            Matrix.Rotation(math.pi / 4, 3, 'Z'), ow)
            # turn around 45 deg
        elif RIGHT in k and FORWARDS in k:
            if active_camera.name == ("Human_Camera"+suffix):
                applyrotate(pos.worldOrientation *
                            Matrix.Rotation(math.pi * 7 / 4, 3, 'Z'), ow)
            else:
                applyrotate(human_pos.worldOrientation *
                            Matrix.Rotation(math.pi * 7 / 4, 3, 'Z'), ow)
            # turn around 315 deg
        elif BACKWARDS in k and not(LEFT in k or RIGHT in k):
            if active_camera.name == ("Human_Camera"+suffix):
                applyrotate(pos.worldOrientation * Matrix.Rotation(math.pi, 3, 'Z'), ow)
            # turn around 180 deg if in game-mode
        elif LEFT in k and BACKWARDS in k:          
            if active_camera.name == ("Human_Camera"+suffix):
                applyrotate(pos.worldOrientation * Matrix.Rotation(math.pi * 3/4, 3, 'Z'), ow)
            else:
                applyrotate(human_pos.worldOrientation * Matrix.Rotation(math.pi / 4, 3, 'Z'), ow)
            # turn around 135 deg if in game-mode, else turn 45 deg
        elif RIGHT in k and BACKWARDS in k:
            if active_camera.name == ("Human_Camera"+suffix):
                applyrotate(pos.worldOrientation * Matrix.Rotation(math.pi * 5/4, 3, 'Z'), ow)
            else:
                applyrotate(human_pos.worldOrientation * Matrix.Rotation(math.pi * 7 / 4, 3, 'Z'), ow)
            # turn around 225 deg if in game mode, else turn 315 deg.


