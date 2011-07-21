import logging; logger = logging.getLogger("morse." + __name__)
######################################################
#
#    MouseLook.py        Blender 2.49
#
#    Modified version of
#    Tutorial for using MouseLook.py found at
#    www.tutorialsforblender3D.com
#
#    Gilberto Echeverria
#    06 / 05 / 2010
#
######################################################

import Rasterizer
import GameLogic
import GameKeys
#import GameTypes

def move(contr):
    """ Read the keys for specific combinations
        that will make the camera move in 3D space. """
    # get the object this script is attached to
    camera = contr.owner

    # set the movement speed
    speed = camera['Speed']

    # Get sensor named Mouse
    keyboard = contr.sensors['All_Keys']

    # Default movement speed
    move_speed = [0.0, 0.0, 0.0]

    keylist = keyboard.events
    for key in keylist:
        # key[0] == GameKeys.keycode, key[1] = status
        if key[1] == GameLogic.KX_INPUT_ACTIVE:
            # Also add the key corresponding key for an AZERTY keyboard
            if key[0] == GameKeys.WKEY or key[0] == GameKeys.ZKEY:
                move_speed[2] = -speed
            elif key[0] == GameKeys.SKEY:
                move_speed[2] = speed
            # Also add the key corresponding key for an AZERTY keyboard
            elif key[0] == GameKeys.AKEY or key[0] == GameKeys.QKEY:
                move_speed[0] = -speed
            elif key[0] == GameKeys.DKEY:
                move_speed[0] = speed
            elif key[0] == GameKeys.RKEY:
                move_speed[1] = speed
            elif key[0] == GameKeys.FKEY:
                move_speed[1] = -speed

            # The second parameter of 'applyMovement' determines
            #  a movement with respect to the object's local
            #  coordinate system
            camera.applyMovement( move_speed, True )

    # Get sensor named Mouse
    #for sensor in contr.sensors:
        #if sensor.isA(GameTypes.SCA_KeyboardSensor):


def rotate(contr):
    """ Read the movements of the mouse and apply them
        as a rotation to the camera. """
    # get the object this script is attached to
    camera = contr.owner

    # Get sensor named Mouse
    mouse = contr.sensors['Mouse']

    if mouse.positive:
        # get width and height of game window
        width = Rasterizer.getWindowWidth()
        height = Rasterizer.getWindowHeight()

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
        Rasterizer.setMousePosition(width//2, height//2)


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
