""" This module wraps the calls to the Blender Python API. This is intended
for all the cases we need to run MORSE code outside Blender (mostly for
documentation generation purposes).
"""

import sys
import os

fake = False

# running in Blender?
# Note: Run blender-app.exe when blender v2.75 in Window 7
if os.path.basename(sys.executable) in ['blender', 'blender.exe', 'blender-app.exe']:
    import bpy
    try:
        import bge
    except ImportError:
        # Can fail if we are in Blender but not yet in the GameEngine,
        # typically at 'Builder' stage.
        fake = True
else:
    print("WARNING: MORSE is running outside Blender! (sys.executable == '%s')" % sys.executable)
    fake = True

from morse.core import mathutils
import logging
logger = logging.getLogger("morse." + __name__)

UPARROWKEY = None
DOWNARROWKEY = None
RIGHTARROWKEY = None
LEFTARROWKEY = None
LEFTCTRLKEY = None
LEFTALTKEY = None

AKEY = None
BKEY = None
DKEY = None
EKEY = None
FKEY = None
GKEY = None
HKEY = None
IKEY = None
JKEY = None
KKEY = None
LKEY = None
NKEY = None
OKEY = None
QKEY = None
RKEY = None
SKEY = None
TKEY = None
UKEY = None
VKEY = None
WKEY = None
XKEY = None
ZKEY = None

LEFTMOUSE = None
RIGHTMOUSE = None

F5KEY = None
F7KEY = None
F8KEY = None

CONSTRAINT_TYPE_KINEMATIC = None
CONSTRAINT_IK_DISTANCE = None

if not fake:
    UPARROWKEY = bge.events.UPARROWKEY
    DOWNARROWKEY = bge.events.DOWNARROWKEY
    RIGHTARROWKEY = bge.events.RIGHTARROWKEY
    LEFTARROWKEY = bge.events.LEFTARROWKEY
    LEFTCTRLKEY = bge.events.LEFTCTRLKEY
    LEFTALTKEY = bge.events.LEFTALTKEY
    AKEY = bge.events.AKEY
    BKEY = bge.events.BKEY
    DKEY = bge.events.DKEY
    EKEY = bge.events.EKEY
    FKEY = bge.events.FKEY
    GKEY = bge.events.GKEY
    HKEY = bge.events.HKEY
    IKEY = bge.events.IKEY
    JKEY = bge.events.JKEY
    KKEY = bge.events.KKEY
    LKEY = bge.events.LKEY
    NKEY = bge.events.NKEY
    OKEY = bge.events.OKEY
    QKEY = bge.events.QKEY
    RKEY = bge.events.RKEY
    SKEY = bge.events.SKEY
    TKEY = bge.events.TKEY
    UKEY = bge.events.UKEY
    VKEY = bge.events.UKEY
    WKEY = bge.events.WKEY
    XKEY = bge.events.XKEY
    ZKEY = bge.events.ZKEY

    LEFTMOUSE = bge.events.LEFTMOUSE
    RIGHTMOUSE = bge.events.RIGHTMOUSE

    F8KEY = bge.events.F8KEY
    F7KEY = bge.events.F7KEY
    F5KEY = bge.events.F5KEY

    CONSTRAINT_TYPE_KINEMATIC = bge.logic.CONSTRAINT_TYPE_KINEMATIC
    CONSTRAINT_IK_DISTANCE = bge.logic.CONSTRAINT_IK_DISTANCE

def input_active():
    if not fake:
        return bge.logic.KX_INPUT_ACTIVE
    else:
        return None

def input_just_activated():
    if not fake:
        return bge.logic.KX_INPUT_JUST_ACTIVATED
    else:
        return None

def input_just_released():
    if not fake:
        return bge.logic.KX_INPUT_JUST_RELEASED
    else:
        return None

def input_none():
    if not fake:
        return bge.logic.KX_INPUT_NONE
    else:
        return None

def keyboard():
    if not fake:
        return bge.logic.keyboard
    else:
        return None

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

def add_scene(name, overlay=1):
    if not fake:
        return bge.logic.addScene(name, overlay)
    else:
        return None

def get_scene_list():
    if not fake:
        return bge.logic.getSceneList()
    else:
        return None

def get_scene_map():
    if not fake:
        return {s.name: s for s in bge.logic.getSceneList()}
    else:
        return None

def render():
    if not fake:
        return bge.render
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


def mousepointer(visible = True):
    if not fake:
        bge.logic.mouse.visible = visible


def constraints():
    if not fake:
        return bge.constraints
    else:
        return None


def texture():
    if not fake:
        return bge.texture
    else:
        return None

def objectdata(name):
    if not fake:
        return bpy.data.objects[name]
    else:
        return None

def materialdata(name):
    if not fake:
        return bpy.data.materials[name]
    else:
        return None

def game_settings():
    if not fake:
        return bpy.context.scene.game_settings
    else:
        return None


def getalwayssensors(obj):
    if not fake:
        return [s for s in obj.sensors if isinstance(s, bge.types.SCA_AlwaysSensor)]
    else:
        return []

def get_armatures(obj):
    if not fake:
        return [child for child in obj.children if isinstance(child, bge.types.BL_ArmatureObject)]
    else:
        return []


def getfrequency():
    if not fake:
        return bge.logic.getLogicTicRate()
    else:
        return 0

def setfrequency(value):
    if not fake:
        return bge.logic.setLogicTicRate(value)
    else:
        return 0

class PersistantStorage(dict):
        __getattr__= dict.__getitem__
        __setattr__= dict.__setitem__
        __delattr__= dict.__delitem__

def persistantstorage():
    """Simply returns the bge.logic object, that persists
    between calls to the script.
    """
    if not fake:
        if not hasattr(bge.logic, "morsedata"):
            bge.logic.morsedata = PersistantStorage()
        return bge.logic.morsedata
    else:
        return {}

def version():
    if not fake:
        return bpy.app.version
    else:
        return 0,0,0

def getssr():
    if not fake:
        return bge.logic.getCurrentScene().objects["Scene_Script_Holder"]
    else:
        return None

def joysticks():
    if not fake:
        return bge.logic.joysticks
    else:
        return None

def isfastmode():
    for area in bpy.context.window.screen.areas:
        if area.type == 'VIEW_3D':
            for space in area.spaces:
                if space.type == 'VIEW_3D':
                    return space.viewport_shade == 'WIREFRAME'

def gravity():
    if not fake:
        sce = scene()
        # All supported version of blender do not support it, so well
        # guess if we don't have the support
        if hasattr(sce, 'gravity'):
            return sce.gravity
        else:
            return mathutils.Vector((0.0, 0.0, -9.81))
    else:
        return None

def clock_time():
    if not fake:
        if hasattr(bge.logic, 'getClockTime'):
            return bge.logic.getClockTime()
        else:
            return -1
    else:
        return -1

def frame_time():
    if not fake:
        if hasattr(bge.logic, 'getFrameTime'):
            return bge.logic.getFrameTime()
        else:
            return -1
    else:
        return -1

def set_time_scale(time_scale):
    if not fake:
        if hasattr(bge.logic, 'setTimeScale'):
            bge.logic.setTimeScale(time_scale)
            return True
        else:
            logger.warning("setTimeScale requires at least Blender 2.77")
    return False
