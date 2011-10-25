from bge import logic

# Cache for the list of active/graspable objects.
# WARNING: the current implementation do not allow
# to change the state active/non active or graspable/not graspable
# during the simulation!
_active_objects = []
_graspable_objects = []

def active_objects():
    """ Returns all active objects in current scene, ie objects that have their
    'Object' property set to True.
    """
    global _active_objects

    if not _active_objects:
        scene = logic.getCurrentScene()
        _active_objects = [obj for obj in scene.objects if 'Object' in obj and obj['Object']]

    return _active_objects

def graspable_objects():
    """ Returns all objects in current scene that have the
    'Graspable' property set to True, amongst active objects.
    """
    global _graspable_objects

    if not _graspable_objects:
        _graspable_objects = [obj for obj in active_objects() if 'Graspable' in obj and obj['Graspable']]

    return _graspable_objects

def details(obj):
    """ Returns a tuple (label, description, type) for a given
    active object.

    If the object is not active (ie, it has no 'Object' property, or the
    property is set to False), None is returned.

    If no label is available, it defaults to the Blender name.
    The description field may be an empty string.
    If no type is available, it defaults to 'Object'.

    :param name: the Blender name of the object.
    :return: a tuple (label, description, type)

    """

    if not obj in active_objects():
        return None
    else:
        return (obj['Label'] if 'Label' in obj else str(obj),
                obj['Description'] if 'Description' in obj else "",
                obj['Type'] if 'Type' in obj else "Object")

def label(obj):
    """ Returns the label of a given active object.

    If the object is not active (ie, it has no 'Object' property, or the
    property is set to False), None is returned.

    If no label is available, it defaults to the Blender name.

    :param name: the Blender name of the object.
    :return: the label

    """

    if not obj in active_objects():
        return None
    else:
        return obj['Label'] if 'Label' in obj else str(obj)
