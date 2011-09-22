import bpy
import math
import morse.builder.morsebuilder

class Cube(morse.builder.morsebuilder.AbstractComponent):
    def __init__(self, name):
        morse.builder.morsebuilder.AbstractComponent.__init__(self)
        bpy.ops.object.select_all(action = 'DESELECT')
        bpy.ops.mesh.primitive_cube_add()
        self._blendobj = bpy.context.selected_objects[0]
        self._blendobj.name = name
        # no collision by default for components
        self._blendobj.game.physics_type = 'NO_COLLISION'

class Cylinder(morse.builder.morsebuilder.AbstractComponent):
    def __init__(self, name):
        morse.builder.morsebuilder.AbstractComponent.__init__(self)
        bpy.ops.object.select_all(action = 'DESELECT')
        bpy.ops.mesh.primitive_cylinder_add()
        self._blendobj = bpy.context.selected_objects[0]
        self._blendobj.name = name
        # no collision by default for components
        self._blendobj.game.physics_type = 'NO_COLLISION'

class Spot(morse.builder.morsebuilder.AbstractComponent):
    def __init__(self, name):
        morse.builder.morsebuilder.AbstractComponent.__init__(self)
        bpy.ops.object.select_all(action = 'DESELECT')
        bpy.ops.object.lamp_add(type='SPOT')
        self._blendobj = bpy.context.selected_objects[0]
        self._blendobj.name = name
        self.rotate(y=-math.pi/2)
        spot = bpy.data.lamps[-1]
        spot.spot_size = math.pi/2
        spot.distance = 10
        # no collision by default for components
        self._blendobj.game.physics_type = 'NO_COLLISION'

class Camera(morse.builder.morsebuilder.AbstractComponent):
    def __init__(self, name):
        morse.builder.morsebuilder.AbstractComponent.__init__(self)
        bpy.ops.object.select_all(action = 'DESELECT')
        bpy.ops.object.camera_add()
        self._blendobj = bpy.context.selected_objects[0]
        self._blendobj.name = name
        # looking in +x
        self.rotate(x=math.pi/2, z=-math.pi/2)
        # no collision by default for components
        self._blendobj.game.physics_type = 'NO_COLLISION'

