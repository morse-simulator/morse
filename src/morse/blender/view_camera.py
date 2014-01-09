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

robots = []
current_robot = 0
# Matrix.Translation((-1, 0, 2)) * Euler((rad(60), 0, rad(-90)), 'XYZ').to_matrix().to_4x4()
camera_to_robot_transform = mathutils.Matrix( (
    ( 0.0, 0.5,  -0.866, -1.0),
    (-1.0, 0.0,   0.0,    0.0),
    ( 0.0, 0.866, 0.5,    2.0),
    ( 0.0, 0.0,   0.0,    1.0) ) )

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

def reset_position(camera):
    """ Put the camera in the initial position and orientation """
    camera.worldPosition = start_position
    camera.worldOrientation = start_orientation

def look_robot(camera):
    """ Put the camera above a robot """
    global robots, current_robot
    if not robots:
        robots = [r for r in blenderapi.persistantstorage().robotDict]

    robot = robots[ current_robot ]
    camera.worldTransform = robot.worldTransform * camera_to_robot_transform
    # switch between robots
    current_robot = (current_robot + 1) % len(robots)


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

    # set camera position increment from the movement speed
    pos_inc = camera['Speed'] / blenderapi.getfrequency()

    # Get Blender keyboard sensor
    keyboard = contr.sensors['All_Keys']

    # Default movement
    move_translation = [0.0, 0.0, 0.0]

    keylist = keyboard.events
    for key in keylist:
        if key[1] == blenderapi.input_active():
            # Also add the corresponding key for an AZERTY keyboard
            if key[0] == blenderapi.WKEY or key[0] == blenderapi.ZKEY:
                move_translation[2] = -pos_inc
            elif key[0] == blenderapi.SKEY:
                move_translation[2] = pos_inc
            # Also add the corresponding key for an AZERTY keyboard
            elif key[0] == blenderapi.AKEY or key[0] == blenderapi.QKEY:
                move_translation[0] = -pos_inc
            elif key[0] == blenderapi.DKEY:
                move_translation[0] = pos_inc
            elif key[0] == blenderapi.RKEY:
                move_translation[1] = pos_inc
            elif key[0] == blenderapi.FKEY:
                move_translation[1] = -pos_inc
            else:
                move_translation[0] = 0
                move_translation[1] = 0
                move_translation[2] = 0

            # The second parameter of 'applyMovement' determines
            #  a movement with respect to the object's local
            #  coordinate system
            camera.applyMovement( move_translation, True )

        elif key[1] == blenderapi.input_just_activated():
            # Other actions activated with the keyboard
            # Reset camera to center
            if key[0] == blenderapi.F8KEY and keyboard.positive:
                reset_position(camera)
            if key[0] == blenderapi.F7KEY and keyboard.positive:
                look_robot(camera)


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
    return x, y
