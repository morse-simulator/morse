import bpy
import morse.builder.morsebuilder

class ComponentCreator(morse.builder.morsebuilder.AbstractComponent):
    def __init__(self, name, calling_module, blendname=None):
        """ ComponentCreator constructor
        
        This class allow to create Python component for MORSE. It consists of an 
        Empty object, to which you can then add meshs of your choice. It adds 
        automaticaly the logic (Always sensor link to a Python controller). And 
        set the default physics_type to 'NO_COLLISION'.

        name: (string) name of the Empty Blender object
        callingModule: (string) in ['calling.actuator_action', 'calling.sensor_action', 'calling.mw_action']
        blendname: (string) used for the middleware configuration (default: None)
            see morse.builder.data.MORSE_MIDDLEWARE_DICT
        """
        morse.builder.morsebuilder.AbstractComponent.__init__(self)
        bpy.ops.object.select_all(action = 'DESELECT')
        bpy.ops.object.add() # default is Empty object
        self._blendobj = bpy.context.selected_objects[0]
        self._blendobj.name = name
        self._blendname = blendname # for middleware configuration
        bpy.ops.object.select_all(action = 'DESELECT')
        bpy.ops.object.select_name(name = self._blendobj.name)
        bpy.ops.logic.sensor_add() # default is Always sensor
        sensor = self._blendobj.game.sensors.keys()[-1]
        self._blendobj.game.sensors[sensor].use_pulse_true_level = True
        bpy.ops.logic.controller_add(type='PYTHON')
        controller = self._blendobj.game.controllers.keys()[-1]
        self._blendobj.game.controllers[controller].mode = 'MODULE'
        self._blendobj.game.controllers[controller].module = calling_module
        self._blendobj.game.controllers[controller].link( sensor = \
                self._blendobj.game.sensors[sensor] )
        # no collision by default for components
        self._blendobj.game.physics_type = 'NO_COLLISION'

class SensorCreator(ComponentCreator):
    def __init__(self, name, class_path, class_name, blendname=None):
        ComponentCreator.__init__(self, name, 'calling.sensor_action', \
                                  blendname)
        self.properties(Component_Tag = True, Class = class_name, \
                Path = class_path)

class ActuatorCreator(ComponentCreator):
    def __init__(self, name, class_path, class_name, blendname=None):
        ComponentCreator.__init__(self, name, 'calling.actuator_action', \
                                  blendname)
        self.properties(Component_Tag = True, Class = class_name, \
                Path = class_path)

