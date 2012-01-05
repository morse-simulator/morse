import os
import bpy
import morse.builder.morsebuilder

def diff_list(l1, l2):
    return [obj for obj in l1 if obj not in l2]

def objects_names():
    return [obj.name for obj in bpy.data.objects]

def collada_importer(filepath):
    l1 = objects_names()
    bpy.ops.wm.collada_import(filepath=filepath)
    return diff_list(objects_names(), l1)

class ComponentCreator(morse.builder.morsebuilder.AbstractComponent):
    def __init__(self, name, calling_module, filename=None):
        """ ComponentCreator constructor

        This class allow to create Python component for MORSE. It consists of an 
        Empty object, to which you can then add meshs of your choice. It adds 
        automaticaly the logic (Always sensor link to a Python controller). And 
        set the default physics_type to 'NO_COLLISION'.

        :param name: (string) name of the Empty object in the Blender scene
        :param calling_module: (string) in ['calling.actuator_action', 'calling.sensor_action', 'calling.mw_action']
        :param filename: (string, optional) used for the middleware configuration
            name of the Blender file in MORSE_COMPONENTS/category/filename.blend
            see morse.builder.data.MORSE_MIDDLEWARE_DICT (default: None)
        :return: a new AbstractComponent instance.
        """
        morse.builder.morsebuilder.AbstractComponent.__init__(self)
        bpy.ops.object.select_all(action = 'DESELECT')
        bpy.ops.object.add() # default is Empty object
        self._blendobj = bpy.context.selected_objects[0]
        self._blendobj.name = name
        self._blendname = filename # for middleware configuration
        bpy.ops.object.select_all(action = 'DESELECT')
        bpy.ops.object.select_name(name = self._blendobj.name)
        bpy.ops.logic.sensor_add() # default is Always sensor
        sensor = self._blendobj.game.sensors[-1]
        sensor.use_pulse_true_level = True
        bpy.ops.logic.controller_add(type='PYTHON')
        controller = self._blendobj.game.controllers[-1]
        controller.mode = 'MODULE'
        controller.module = calling_module
        controller.link(sensor = sensor)
        # no collision by default for components
        self._blendobj.game.physics_type = 'NO_COLLISION'
    def frequency(self, delay=0):
        """ Set the frequency delay for the call of the Python module

        :param delay: (int) Delay between repeated pulses 
            (in logic tics, 0 = no delay)
        """
        sensor = self._blendobj.game.sensors['Always']
        sensor.frequency = delay
    def append_meshes(self, objects, component=None):
        """ Append the objects to this component

        The `objects` are located in:
        MORSE_COMPONENTS/`self.category`/`component`.blend/Object/

        :param objects: list of the objects names to append
        :param component: component in which the objects are located
        :return: list of the selected Blender objects
        """
        if not component:
            component = self._blendname
        filepath = os.path.join(morse.builder.morsebuilder.MORSE_COMPONENTS, \
                                self.category, component + '.blend')
        objlist = [{'name':obj} for obj in objects]
        bpy.ops.object.select_all(action='DESELECT')
        # append the objects to the scene, and (auto)select them
        bpy.ops.wm.link_append(directory=filepath + '/Object/', link=False, 
                autoselect=True, files=objlist)
        # make the objects children of the component
        for child in bpy.context.selected_objects:
            if not child.parent:
                child.parent = self._blendobj
        return bpy.context.selected_objects
    def append_collada(self, component=None):
        """ Append Collada objects to this component

        The objects are located in:
        MORSE_COMPONENTS/`self.category`/`component`.dae

        :param component: component in which the objects are located
        :return: list of the selected Blender objects
        """
        if not component:
            component = self._blendname
        filepath = os.path.join(morse.builder.morsebuilder.MORSE_COMPONENTS, \
                                self.category, component + '.dae')
        return self._append_collada_filepath(filepath)
    def _append_collada_filepath(self, filepath):
        imported_objects = collada_importer(filepath)
        for childname in imported_objects:
            child = bpy.data.objects[childname]
            if not child.parent:
                child.parent = self._blendobj
        return imported_objects

class SensorCreator(ComponentCreator):
    def __init__(self, name, class_path, class_name, blendname=None):
        ComponentCreator.__init__(self, name, 'calling.sensor_action', \
                                  blendname)
        self.properties(Component_Tag = True, Class = class_name, \
                Path = class_path)
        self.category = "sensors"

class ActuatorCreator(ComponentCreator):
    def __init__(self, name, class_path, class_name, blendname=None):
        ComponentCreator.__init__(self, name, 'calling.actuator_action', \
                                  blendname)
        self.properties(Component_Tag = True, Class = class_name, \
                Path = class_path)
        self.category = "actuators"

class RobotCreator(ComponentCreator):
    def __init__(self, name, class_path, class_name, blendname=None):
        ComponentCreator.__init__(self, name, 'calling.robot_action', \
                                  blendname)
        self.properties(Robot_Tag = True, Class = class_name, \
                Path = class_path)
        self.category = "robots"

