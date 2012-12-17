import logging; logger = logging.getLogger("morse." + __name__)
from morse.core import blenderapi


def active_objects():
    """ Returns all active objects in current scene, ie objects that have their
    'Object' property set to True.
    """
    return blenderapi.persistantstorage().passiveObjectsDict.keys()

def graspable_objects():
    """ Returns all objects in current scene that have the
    'Graspable' property set to True, amongst active objects.
    """
    return [obj for (obj, details) in blenderapi.persistantstorage().passiveObjectsDict.items() if details['graspable']]

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
    if not obj in blenderapi.persistantstorage().passiveObjectsDict.keys():
        return None
    else:
        return blenderapi.persistantstorage().passiveObjectsDict[obj]

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
    for obj, det in blenderapi.persistantstorage().passiveObjectsDict.items():
        if det['label'] == label:
            return obj
