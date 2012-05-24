import logging; logger = logging.getLogger("morse." + __name__)
######################################################
#
#    view_camera.py        Blender 2.59
#
#    Modified version of
#    Tutorial for using MouseLook.py found at
#    www.tutorialsforblender3D.com
#
#    Gilberto Echeverria
#    15 / 09 / 2010
#
######################################################

import bge
import mathutils

start_position = []
start_orientation = []

def store_default(contr):
    """ Save the initial position and orientation of the camera """
    global start_position
    global start_orientation

    camera = contr.owner
    # Create a copy of the current positions
    start_position = mathutils.Vector(camera.worldPosition)
    start_orientation = mathutils.Matrix(camera.worldOrientation)


def reset_position(contr):
    """ Put the camera in the initial position and orientation """
    camera = contr.owner
    camera.worldPosition = start_position
    camera.worldOrientation = start_orientation


def move(contr):
    """ Read the keys for specific combinations
        that will make the camera move in 3D space. """
    # get the object this script is attached to
    camera = contr.owner

    scene = bge.logic.getCurrentScene()

    # Do not move the camera if the current view is using another camera
    if camera != scene.active_camera:
        return

    if 'Human'  in scene.objects:
        human = scene.objects['Human']
        if not human['move_cameraFP']:
            return

    # set the movement speed
    speed = camera['Speed']

    # Get Blender keyboard sensor
    keyboard = contr.sensors['All_Keys']

    # Default movement speed
    move_speed = [0.0, 0.0, 0.0]

    keylist = keyboard.events
    for key in keylist:
        # key[0] == bge.events.keycode, key[1] = status
        if key[1] == bge.logic.KX_INPUT_ACTIVE:
            # Also add the corresponding key for an AZERTY keyboard
            if key[0] == bge.events.WKEY or key[0] == bge.events.ZKEY:
                move_speed[2] = -speed
            elif key[0] == bge.events.SKEY:
                move_speed[2] = speed
            # Also add the corresponding key for an AZERTY keyboard
            elif key[0] == bge.events.AKEY or key[0] == bge.events.QKEY:
                move_speed[0] = -speed
            elif key[0] == bge.events.DKEY:
                move_speed[0] = speed
            elif key[0] == bge.events.RKEY:
                move_speed[1] = speed
            elif key[0] == bge.events.FKEY:
                move_speed[1] = -speed
            else:
                move_speed[0] = 0
                move_speed[1] = 0
                move_speed[2] = 0

            # The second parameter of 'applyMovement' determines
            #  a movement with respect to the object's local
            #  coordinate system
            camera.applyMovement( move_speed, True )

        elif key[1] == bge.logic.KX_INPUT_JUST_ACTIVATED:
            # Other actions activated with the keyboard
            # Reset camera to center
            if key[0] == bge.events.F8KEY and keyboard.positive:
                reset_position(contr)


def rotate(contr):
    """ Read the movements of the mouse and apply them
        as a rotation to the camera. """
    # get the object this script is attached to
    camera = contr.owner

    # Do not move the camera if the current view is using another camera
    if camera != bge.logic.getCurrentScene().active_camera:
        return

    # Get sensor named Mouse
    mouse = contr.sensors['Mouse']
    # Get Blender keyboard sensor
    keyboard = contr.sensors['All_Keys']

    # Show the cursor
    mouse_visible = True

    keylist = keyboard.events
    for key in keylist:
        # key[0] == bge.events.keycode, key[1] = status
        if key[1] == bge.logic.KX_INPUT_ACTIVE:
            # Left CTRL key allow to rotate the camera
            if key[0] == bge.events.LEFTCTRLKEY:
                # Hide the cursor while we control the camera
                mouse_visible = False
                if mouse.positive:
                    # get width and height of game window
                    width = bge.render.getWindowWidth()
                    height = bge.render.getWindowHeight()

                    # get mouse movement from function
                    move = mouse_move(camera, mouse, width, height)

                    # set mouse sensitivity
                    sensitivity = camera['Sensitivity']

                    # Amount, direction and sensitivity
                    leftRight = move[0] * sensitivity
                    upDown = move[1] * sensitivity

                    # set the values
                    camera.applyRotation( [0.0, 0.0, leftRight], 0 )
                    camera.applyRotation( [upDown, 0.0, 0.0], 1 )

                    # Center mouse in game window
                    # Using the '//' operator (floor division) to produce an integer result
                    bge.render.setMousePosition(width//2, height//2)

    # Set the cursor visibility
    bge.logic.mouse.visible = mouse_visible

def mouse_move(camera, mouse, width, height):
    """ Get the movement of the mouse as an X, Y coordinate. """
    # distance moved from screen center
    # Using the '//' operator (floor division) to produce an integer result
    x = width//2 - mouse.position[0]
    y = height//2 - mouse.position[1]
    
    # intialize mouse so it doesn't jerk first time
    try:
        camera['mouseInit']
    except KeyError:
        x = 0
        y = 0
        # bug in Add Property
        # can't use True.  Have to use 1
        camera['mouseInit'] = 1

    logger.debug("Read displacement: %s, %s" % (x, y))
    
    # return mouse movement
    return (x, y)
