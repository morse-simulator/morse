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

import mathutils
from morse.core import blenderapi

start_position = []
start_orientation = []
keyboard_ctrl_objects = []

def store_default(contr):
    """ Save the initial position and orientation of the camera """
    global start_position
    global start_orientation

    camera = contr.owner
    # Create a copy of the current positions
    start_position = mathutils.Vector(camera.worldPosition)
    start_orientation = mathutils.Matrix(camera.worldOrientation)

    # look for objects that define the move_cameraFP property to
    # disable keyboard control of the camera
    scene = blenderapi.scene()
    if not scene:
        # not ready, main reload(blenderapi)
        return
    for obj in scene.objects:
        if 'move_cameraFP' in obj.getPropertyNames():
            keyboard_ctrl_objects.append(obj)

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

    scene = blenderapi.scene()
    if not scene:
        # not ready, main reload(blenderapi)
        return

    # Do not move the camera if the current view is using another camera
    if camera != scene.active_camera:
        return

    # Do not move the camera if another object has set move_cameraFP
    for obj in keyboard_ctrl_objects:
        if not obj['move_cameraFP']:
            return

    # set the movement speed
    speed = camera['Speed']

    # Get Blender keyboard sensor
    keyboard = contr.sensors['All_Keys']

    # Default movement speed
    move_speed = [0.0, 0.0, 0.0]

    keylist = keyboard.events
    for key in keylist:
        if key[1] == blenderapi.input_active():
            # Also add the corresponding key for an AZERTY keyboard
            if key[0] == blenderapi.WKEY or key[0] == blenderapi.ZKEY:
                move_speed[2] = -speed
            elif key[0] == blenderapi.SKEY:
                move_speed[2] = speed
            # Also add the corresponding key for an AZERTY keyboard
            elif key[0] == blenderapi.AKEY or key[0] == blenderapi.QKEY:
                move_speed[0] = -speed
            elif key[0] == blenderapi.DKEY:
                move_speed[0] = speed
            elif key[0] == blenderapi.RKEY:
                move_speed[1] = speed
            elif key[0] == blenderapi.FKEY:
                move_speed[1] = -speed
            else:
                move_speed[0] = 0
                move_speed[1] = 0
                move_speed[2] = 0

            # The second parameter of 'applyMovement' determines
            #  a movement with respect to the object's local
            #  coordinate system
            camera.applyMovement( move_speed, True )

        elif key[1] == blenderapi.input_just_activated():
            # Other actions activated with the keyboard
            # Reset camera to center
            if key[0] == blenderapi.F8KEY and keyboard.positive:
                reset_position(contr)


def rotate(contr):
    """ Read the movements of the mouse and apply them
        as a rotation to the camera. """
    # get the object this script is attached to
    camera = contr.owner

    scene = blenderapi.scene()
    if not scene:
        # not ready, main reload(blenderapi)
        return

    # Do not move the camera if the current view is using another camera
    if camera != scene.active_camera:
        return

    # Get sensor named Mouse
    mouse = contr.sensors['Mouse']
    # Get Blender keyboard sensor
    keyboard = contr.sensors['All_Keys']

    # Show the cursor
    mouse_visible = True

    keylist = keyboard.events
    for key in keylist:
        if key[1] == blenderapi.input_active():
            # Left CTRL key allow to rotate the camera
            if key[0] == blenderapi.LEFTCTRLKEY:
                # Hide the cursor while we control the camera
                mouse_visible = False
                if mouse.positive:
                    # get width and height of game window
                    width = blenderapi.render().getWindowWidth()
                    height = blenderapi.render().getWindowHeight()

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
                    blenderapi.render().setMousePosition(width//2, height//2)

    # Set the cursor visibility
    blenderapi.mousepointer(visible = mouse_visible)

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
