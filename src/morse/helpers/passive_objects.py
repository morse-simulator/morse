import logging; logger = logging.getLogger("morse." + __name__)
import bge


def active_objects():
    """ Returns all active objects in current scene, ie objects that have their
    'Object' property set to True.
    """

    if not bge.logic.passiveObjectsDict:
        logger.error("Initialization error! the passive objects dictionary has not been built!")
        return {}
        #return None

    return bge.logic.passiveObjectsDict.keys()

def graspable_objects():
    """ Returns all objects in current scene that have the
    'Graspable' property set to True, amongst active objects.
    """

    if not bge.logic.passiveObjectsDict:
        logger.error("Initialization error! the passive objects dictionary has not been built!")
        return None

    return [obj for (obj, details) in bge.logic.passiveObjectsDict.items() if details['graspable']]

def details(obj):
    """ Returns a dictionary containing the differents properties for a given
    active object.

    If the object is not active (ie, it has no 'Object' property, or the
    property is set to False), None is returned.

    If no label is available, it defaults to the Blender name.
    The description field may be an empty string.
    If no type is available, it defaults to 'Object'.
    If the graspable flag is not present, it defaults to False.

    :param name: the Blender name of the object.
    :return: a dictionary {'label':string, 'description':string, 'type':string, 'graspable':bool}

    """

    if not bge.logic.passiveObjectsDict:
        logger.error("Initialization error! the passive objects dictionary has not been built!")
        return None

    if not obj in bge.logic.passiveObjectsDict.keys():
        return None
    else:
        return bge.logic.passiveObjectsDict[obj]

def label(obj):
    """ Returns the label of a given active object.

    If the object is not active (ie, it has no 'Object' property, or the
    property is set to False), None is returned.

    If no label is available, it defaults to the Blender name.

    :param name: the Blender name of the object.
    :return: the label

    """

    det = details(obj)
    return det['label'] if det else None

def obj_from_label(label):
    """ Returns the label of a given active object.

    If the object is not active (ie, it has no 'Object' property, or the
    property is set to False), None is returned.

    If no label is available, it defaults to the Blender name.

    :param name: the Blender name of the object.
    :return: the label

    """

    if not bge.logic.passiveObjectsDict:
        logger.error("Initialization error! the passive objects dictionary has not been built!")
        return None

    for obj, det in bge.logic.passiveObjectsDict.items():
        if det['label'] == label:
            return obj
