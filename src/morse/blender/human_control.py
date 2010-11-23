
######################################################
#
#    human_control.py        Blender 2.49
#
#    Modified version of
#      view_camera.py by Gilberto Echeverria
#
#    Severin Lemaignan
#    19 / 08 / 2010
#
######################################################

import Rasterizer
import GameLogic
import GameKeys
import math
if sys.version_info<(3,0,0):
    import Mathutils as mathutils
else:
    import mathutils

# Defined in degrees, but used in radians
MAX_HEAD_PAN = math.radian(60.0)
MAX_HEAD_TILT = math.radian(40.0)
MAX_HEAD_YAW = math.radian(10.0)

def move(contr):
    """ Read the keys for specific combinations
        that will make the camera move in 3D space. """
    # get the object this script is attached to
    hum = contr.owner

    # set the movement speed
    speed = hum['Speed']

    # Get sensor named Mouse
    keyboard = contr.sensors['All_Keys']

    # Default movement speed
    move_speed = [0.0, 0.0, 0.0]
    rotation_speed = [0.0, 0.0, 0.0]

    keylist = keyboard.events
    for key in keylist:
        # key[0] == GameKeys.keycode, key[1] = status
        if key[1] == GameLogic.KX_INPUT_ACTIVE:
            if key[0] == GameKeys.UPARROWKEY:
                move_speed[0] = speed
            elif key[0] == GameKeys.DOWNARROWKEY:
                move_speed[0] = -speed
            elif key[0] == GameKeys.LEFTARROWKEY:
                rotation_speed[2] = speed
            elif key[0] == GameKeys.RIGHTARROWKEY:
                rotation_speed[2] = -speed
            elif key[0] == GameKeys.AKEY:
                move_speed[2] = speed
            elif key[0] == GameKeys.EKEY:
                move_speed[2] = -speed

            # The second parameter of 'applyMovement' determines
            #  a movement with respect to the object's local
            #  coordinate system
            hum.applyMovement( move_speed, True )

            hum.applyRotation( rotation_speed, True )


def rotate(contr):
    """ Read the movements of the mouse and apply them
        as a rotation to the human head and camera. """
    # get the object this script is attached to
    hum = contr.owner

    # set the movement speed
    speed = hum['Speed']

    head = hum.children['OBHead']

    # Get sensor named Mouse
    mouse = contr.sensors['Mouse']

    """
    activateHeadMovement = contr.sensors['rmb']

    #If the mouse left button is pushed (to pick an object), don't move the head
    if activateHeadMovement.positive:
        return
    """

    # get width and height of game window
    width = Rasterizer.getWindowWidth()
    height = Rasterizer.getWindowHeight()

    # get mouse movement from function
    move = mouse_move(hum, mouse, width, height)

    # set mouse sensitivity
    sensitivity = hum['Sensitivity']

    # Amount, direction and sensitivity
    leftRight = move[0] * sensitivity
    upDown = move[1] * sensitivity

    orientation = head.orientation
    parent_ori = head.parent.orientation

    ori_mat = mathutils.Matrix(orientation[0], orientation[1], orientation[2])
    parent_ori_mat = mathutils.Matrix(parent_ori[0], parent_ori[1], parent_ori[2])

    head2parent = parent_ori_mat * ori_mat.invert()

    ori_eul_parent = ori_mat.toEuler()
    ori_eul = ori_mat.toEuler()


    print str(ori_eul)

    # set the values
    if abs(ori_eul.z + leftRight) < MAX_HEAD_PAN or abs(ori_eul.z + leftRight) > abs(ori_eul.z):
        ori_eul.z -= leftRight

    if abs(ori_eul.y + upDown) < MAX_HEAD_TILT or abs(ori_eul.y + upDown) > abs(ori_eul.y):
        ori_eul.y -= upDown

    if ori_eul.x > MAX_HEAD_YAW:
        ori_eul.x = MAX_HEAD_YAW
    if ori_eul.x < -MAX_HEAD_YAW:
        ori_eul.x = -MAX_HEAD_YAW

    ori_mat = ori_eul.toMatrix()
    head.setOrientation(ori_mat)
    # Center mouse in game window
    Rasterizer.setMousePosition(width//2, height//2)


# define mouse movement function
def mouse_move(hum, mouse, width, height):
    """ Get the movement of the mouse as an X, Y coordinate. """
    # distance moved from screen center
    x = width/2 - mouse.position[0]
    y = height/2 - mouse.position[1]

    # intialize mouse so it doesn't jerk first time
    try:
        hum['mouseInit']
    except KeyError:
        x = 0
        y = 0
        # bug in Add Property
        # can't use True.  Have to use 1
        hum['mouseInit'] = 1

    # return mouse movement
    return (x, y)
