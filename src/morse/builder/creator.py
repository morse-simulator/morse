import os
import bpy
from morse.builder import AbstractComponent, MORSE_COMPONENTS

class ComponentCreator(AbstractComponent):
    def __init__(self, cname, category, filename=''):
        """ ComponentCreator constructor

        This class allow to create Python component for MORSE. It consists of an
        Empty object, to which you can then add meshs of your choice. It adds
        automaticaly the logic (Always sensor link to a Python controller). And
        set the default physics_type to 'NO_COLLISION'.

        :param cname: (string) component name (Empty object in Blender scene)
        :param category: (string) in ['actuators', 'sensors', 'robots']
        :param filename: (string, optional) used for the datastream configuration
            name of the Blender file in MORSE_COMPONENTS/category/filename.blend
            see morse.builder.data.MORSE_DATASTREAM_DICT (default: None)
        :return: a new AbstractComponent instance.
        """
        AbstractComponent.__init__(self, filename=filename, category=category)
        bpy.ops.object.select_all(action='DESELECT')
        bpy.ops.object.add(type='EMPTY')
        # bpy.ops.object.empty_add(type='ARROWS')
        obj = bpy.context.selected_objects[0]
        obj.name = cname
        # no collision by default for components
        obj.game.physics_type = 'NO_COLLISION'
        self.set_blender_object(obj)
        # Add MORSE logic
        self.morseable()

    def parent_root(self, objects):
        # Parent the root objects with this Component
        for child in objects:
            if not child.parent:
                child.matrix_parent_inverse.identity()
                child.parent = self._blendobj

    def append_meshes(self, objects=None, component=None):
        """ Append the objects to this component

        The `objects` are located in:
        MORSE_COMPONENTS/`self._category`/`component`.blend/Object/

        :param objects: list of the objects names to append
        :param component: component in which the objects are located
        :return: list of the imported Blender objects
        """
        imported_objects = AbstractComponent.append_meshes(self, objects, \
                                                           component)
        self.parent_root(imported_objects)

        return imported_objects

    def append_collada(self, component=None):
        """ Append Collada objects to this component

        The objects are located in:
        MORSE_COMPONENTS/`self._category`/`component`.dae

        :param component: component in which the objects are located
        :return: list of the imported Blender objects
        """
        imported_objects = AbstractComponent.append_collada(self, component)
        self.parent_root(imported_objects)

        return imported_objects

class SensorCreator(ComponentCreator):
    def __init__(self, name="SensorCreator", class_path="morse/core/sensor", \
                 class_name="Sensor", blendname=None):
        ComponentCreator.__init__(self, name, 'sensors', blendname)
        self.properties(Component_Tag = True, Class = class_name, \
                        Path = class_path)

class ActuatorCreator(ComponentCreator):
    def __init__(self, name="ActuatorCreator", class_path="morse/core/actuator", \
                 class_name="Actuator", blendname=None):
        ComponentCreator.__init__(self, name, 'actuators', blendname)
        self.properties(Component_Tag = True, Class = class_name, \
                        Path = class_path)

class RobotCreator(ComponentCreator):
    def __init__(self, name="RobotCreator", class_path="morse/core/robot", \
                 class_name="Robot", blendname=None):
        ComponentCreator.__init__(self, name, 'robots', blendname)
        self.properties(Robot_Tag = True, Class = class_name, \
                        Path = class_path)


# helpers

def get_properties_str(name):
    """ Returns the Game properties of the Blender object represented by the name

    get_properties_str('Sick') gives
    laser_range = 30.0, Component_Tag = True, scan_window = 180.0, 
    Visible_arc = True, Path = 'morse/sensors/sick', resolution = 0.25,
    Class = 'LaserScannerClass'
    """
    obj = bpy.data.objects[name]
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
