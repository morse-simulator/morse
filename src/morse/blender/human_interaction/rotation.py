import bge
from bge import logic
import math
from mathutils import Matrix

def applyrotate(destOr, owner):
    """
    Rotates the owner of this script to the given orientation.
    'smoothness' defines the speed of this rotation
    """
    smoothness = 10
    currOr = owner.worldOrientation
    dZ = [0.0,0.0,0.0]
    for x in range(0, smoothness):
        dZ = currOr.to_euler()[2] - destOr.to_euler()[2]
        #Blender allows multiples of 360 deg and negative angles - this is to get rid of those
        while(dZ < math.pi):
            dZ = dZ + 2 * math.pi
        while(dZ > math.pi):
            dZ = dZ - 2 * math.pi
        owner.worldOrientation = owner.worldOrientation * Matrix.Rotation(-dZ/(10*smoothness), 3, 'Z')

def rotate(co):
    ow = co.owner
    keyboard = co.sensors['Keyboard']
    pos =  logic.getCurrentScene().objects['POS_EMPTY']

    keylist = keyboard.events

    k = []    #initiate a list with all currently pressed keys
    for key in keylist:
        if key[1] ==  logic.KX_INPUT_ACTIVE:
            k.append(key[0])        # add all pressed keys to a list - as ASCII CODES

    ow.worldPosition = pos.worldPosition

    if not pos['Manipulate']:
        if bge.events.WKEY in k and not(bge.events.AKEY in k or bge.events.DKEY in k):  
            applyrotate(pos.worldOrientation, ow)
        elif bge.events.AKEY in k and not(bge.events.WKEY in k or bge.events.SKEY in k):
            applyrotate(pos.worldOrientation * Matrix.Rotation(math.pi / 2, 3, 'Z'), ow)     # turn around 90 deg
        elif bge.events.DKEY in k and not(bge.events.WKEY in k or bge.events.SKEY in k):
            applyrotate(pos.worldOrientation * Matrix.Rotation(math.pi * 3/2, 3, 'Z'), ow)    # turn around 270 deg
        elif bge.events.AKEY in k and bge.events.WKEY in k:
            applyrotate(pos.worldOrientation * Matrix.Rotation(math.pi / 4, 3, 'Z'), ow)  # turn around 45 deg
        elif bge.events.DKEY in k and bge.events.WKEY in k:
            applyrotate(pos.worldOrientation * Matrix.Rotation(math.pi * 7 / 4, 3, 'Z'), ow)    # turn around 315 deg
        elif bge.events.SKEY in k and not(bge.events.AKEY in k or bge.events.DKEY in k):
            applyrotate(pos.worldOrientation * Matrix.Rotation(math.pi, 3, 'Z'), ow)    # turn around 180 deg
        elif bge.events.AKEY in k and bge.events.SKEY in k:
            applyrotate(pos.worldOrientation * Matrix.Rotation(math.pi * 3/4, 3, 'Z'), ow)     # turn around 135 deg
        elif bge.events.DKEY in k and bge.events.SKEY in k:
            applyrotate(pos.worldOrientation * Matrix.Rotation(math.pi * 5/4, 3, 'Z'), ow)    # turn around 225 deg
