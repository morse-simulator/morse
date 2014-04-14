import logging; logger = logging.getLogger("morsebuilder." + __name__)
from morse.builder import AbstractComponent, bpymorse
from morse.core.exceptions import *

class ComponentCreator(AbstractComponent):
    _classpath = None
    _blendname = None

    APPEND_EMPTY = 0
    USE_BLEND = 1
    LINK_EXISTING_OBJECT = 2

    def __init__(self, 
                 name, 
                 category, 
                 action = APPEND_EMPTY, 
                 blendfile = "", 
                 blendobject = None,
                 make_morseable = True):
        """ ComponentCreator constructor

        This class allow to create simulation components from MORSE builder
        scripts. It initially consists in an Empty object, to which you can
        then add meshs of your choice. It adds automaticaly the logic (Always
        sensor link to a Python controller). And set the default physics_type
        to 'NO_COLLISION'.

        :param name: (string) component name (used as Blender object name)
        :param category: (string) one of ['actuators', 'sensors', 'robots']
        :param action: indicate what to do with the `blendfile` and
        `blendobject` parameters. Must be one of [APPEND_EMPTY, USE_BLEND,
        LINK_EXISTING_OBJECT]. 
            - If APPEND_EMPTY (default), a new Blender `Empty` is created and
            `blendfile` and `blendobject` are ignored.
            - If USE_BLEND, `blendfile` is treated as the path to a Blender file,
            and if `blendobject` is also specified, the given object is
            selected (otherwise, the last object selected in the Blender file
            is returned).
            - If LINK_EXISTING_OBJECT, `blendfile` is ignored and `blendobject`
            is treated as the name of a Blender object which is already present
            in the scene.
        :param blendfile: (string, default:"") path to a Blender file (.blend)
        containing meshes for the component. Must be in MORSE_RESOURCE_PATH.
        :param blendobject: (string, default:None) Name of the Blender object
        to use (cf above for details).
        :param make_morseable: (boolean) Add Morse logic. Make it false
            if you add some blend file which already contains the
            necessary logic (default: True).
        """
        AbstractComponent.__init__(self, filename=blendfile, category=category)
        bpymorse.deselect_all()
        if action == ComponentCreator.APPEND_EMPTY:
            bpymorse.add_morse_empty()
        elif action == ComponentCreator.USE_BLEND:
            self.append_meshes()
            if blendobject:
                bpymorse.select_only(bpymorse.get_object(blendobject))
        elif action == ComponentCreator.LINK_EXISTING_OBJECT:
            bpymorse.select_only(bpymorse.get_object(blendobject))

        obj = bpymorse.get_first_selected_object()
        if name:
            obj.name = name
            self.basename = name
        # no collision by default for components
        obj.game.physics_type = 'NO_COLLISION'
        self.set_blender_object(obj)
        # Add MORSE logic
        if make_morseable:
            self.morseable()

        self.properties(Component_Tag = True, classpath = self.__class__._classpath)

    def parent_root(self, objects):
        # Parent the root objects with this Component
        for child in objects:
            if not child.parent:
                child.matrix_parent_inverse.identity()
                child.parent = self._bpy_object

    def append_meshes(self, objects=None, component=None, prefix=None):
        """ Append the objects to this component

        Overloads :py:meth:`morse.builder.abstractcomponent.AbstractComponent.append_meshes`

        :param objects: list of the objects names to append
        :param component: component in which the objects are located
        :return: list of the imported Blender objects
        """
        imported_objects = AbstractComponent.append_meshes(self, objects,
                                                           component, prefix)
        self.parent_root(imported_objects)

        return imported_objects

    def append_collada(self, component=None):
        """ Append Collada objects to this component

        Overloads :py:meth:`morse.builder.abstractcomponent.AbstractComponent.append_collada`

        :param component: component in which the objects are located
        :return: list of the imported Blender objects
        """
        imported_objects = AbstractComponent.append_collada(self, component)
        self.parent_root(imported_objects)

        return imported_objects

class SensorCreator(ComponentCreator):
    def __init__(self, name="SensorCreator",
                       action = ComponentCreator.APPEND_EMPTY,
                       make_morseable = True):

        ComponentCreator.__init__(self, 
                                  name, 
                                  'sensors',
                                  action,
                                  blendfile = self.__class__._blendname, 
                                  make_morseable = make_morseable)


class ActuatorCreator(ComponentCreator):

    def __init__(self, name="ActuatorCreator", 
                       action = ComponentCreator.APPEND_EMPTY,
                       blendfile = None,
                       blendobject = None,
                       make_morseable = True):

        if not blendfile:
            blendfile = self.__class__._blendname

        ComponentCreator.__init__(self, 
                                name, 
                                'actuators', 
                                action, 
                                blendfile,
                                blendobject,
                                make_morseable)



# helpers

def get_properties_str(name):
    """ Returns the Game properties of the Blender object represented by the name

    get_properties_str('Sick') gives
    laser_range = 30.0, Component_Tag = True, scan_window = 180.0, 
    Visible_arc = True, resolution = 0.25,
    classpath = 'morse.sensors.sick.LaserScannerClass'
    """
    obj = bpymorse.get_object(name)
    properties_dictionary = get_properties(obj)
    return ", ".join(["%s = %s"%(pname, properties_dictionary[pname]) \
                      for pname in properties_dictionary])

def get_properties(obj):
    import json
    properties_dictionary = {}
    properties = obj.game.properties
    for name in properties.keys():
        properties_dictionary[name] = properties[name].value
        if properties[name].type == 'TIMER':
            properties_dictionary[name] = "timer(%f)"%properties_dictionary[name]
        elif type(properties_dictionary[name]) is str:
            properties_dictionary[name] = json.dumps(properties_dictionary[name])
    return properties_dictionary
