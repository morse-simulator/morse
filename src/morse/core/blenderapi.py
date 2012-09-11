""" This module wraps the calls to the Blender Python API. This is intended
for all the cases we need to run MORSE code outside Blender (mostly for
documentation generation purposes).
"""

import morse

fake = False

if morse.running_in_blender:
    import bge
else:
    fake = True

UPARROWKEY = None
DOWNARROWKEY = None
RIGHTARROWKEY = None
LEFTARROWKEY = None

if not fake:
    UPARROWKEY = bge.events.UPARROWKEY
    DOWNARROWKEY = bge.events.DOWNARROWKEY
    RIGHTARROWKEY = bge.events.RIGHTARROWKEY
    LEFTARROWKEY = bge.events.LEFTARROWKEY

def controller():
    if not fake:
        return bge.logic.getCurrentController()
    else:
        return None

def scene():
    if not fake:
        return bge.logic.getCurrentScene()
    else:
        return None

def hascameras():
    if not fake:
        return hasattr(bge.logic, 'cameras')
    else:
        return None


def initcameras():
    if not fake:
        bge.logic.cameras = {}


def cameras():
    if not fake:
        return bge.logic.cameras
    else:
        return None


def texture():
    if not fake:
        return bge.texture
    else:
        return None


def getalwayssensors(obj):
    if not fake:
        return [s for s in obj.sensors if isinstance(s, bge.types.SCA_AlwaysSensor)]
    else:
        return []

def getfrequency():
    if not fake:
        return bge.logic.getLogicTicRate()
    else:
        return 0

def persistantstorage():
    """Simply returns the bge.logic object, that persists
    between calls to the script.
    """
    if not fake:
        return bge.logic
    else:
        return None

def version():
    if not fake:
        return bge.logic.blenderVersion 
    else:
        return (0,0,0)


