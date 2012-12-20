import bpy
import math
from morse.builder import AbstractComponent

class Mesh(AbstractComponent):
    mesh_primitive_add = bpy.ops.mesh.primitive_monkey_add # you must set it in subclasses
    def __init__(self, name):
        AbstractComponent.__init__(self)
        bpy.ops.object.select_all(action = 'DESELECT')
        self.mesh_primitive_add()
        obj = bpy.context.selected_objects[0]
        obj.name = name
        # no collision by default for components
        obj.game.physics_type = 'NO_COLLISION'
        self.set_blender_object(obj)

    def color(self, r=0.1, g=0.1, b=0.1):
        if not self._blendobj.active_material:
            bpy.ops.material.new()
            # link material to object
            self._blendobj.active_material = bpy.data.materials[-1]
        self._blendobj.active_material.diffuse_color = (r, g, b)

class Plane(Mesh):
    mesh_primitive_add = bpy.ops.mesh.primitive_plane_add

class Cube(Mesh):
    mesh_primitive_add = bpy.ops.mesh.primitive_cube_add

class Sphere(Mesh):
    mesh_primitive_add = bpy.ops.mesh.primitive_uv_sphere_add

class IcoSphere(Mesh):
    mesh_primitive_add = bpy.ops.mesh.primitive_ico_sphere_add

class Cylinder(Mesh):
    mesh_primitive_add = bpy.ops.mesh.primitive_cylinder_add

class Cone(Mesh):
    mesh_primitive_add = bpy.ops.mesh.primitive_cone_add

class Torus(Mesh):
    mesh_primitive_add = bpy.ops.mesh.primitive_torus_add

class Spot(AbstractComponent):
    def __init__(self, name, lamp_type='SPOT'):
        AbstractComponent.__init__(self)
        bpy.ops.object.select_all(action='DESELECT')
        bpy.ops.object.lamp_add(type=lamp_type)
        obj = bpy.context.selected_objects[0]
        obj.name = name
        # no collision by default for components
        obj.game.physics_type = 'NO_COLLISION'
        self.set_blender_object(obj)
        # Emit in +X
        self.rotate(y=-math.pi/2)
        if lamp_type is 'SPOT':
            spot = bpy.data.lamps[-1]
            spot.spot_size = math.pi/2
            spot.distance = 10

class Camera(AbstractComponent):
    def __init__(self, name):
        AbstractComponent.__init__(self)
        bpy.ops.object.select_all(action='DESELECT')
        bpy.ops.object.camera_add()
        obj = bpy.context.selected_objects[0]
        obj.name = name
        # no collision by default for components
        obj.game.physics_type = 'NO_COLLISION'
        self.set_blender_object(obj)
        # looking in +x
        self.rotate(x=math.pi/2, z=-math.pi/2)
