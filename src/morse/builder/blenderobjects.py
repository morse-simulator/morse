import math
from morse.builder import AbstractComponent, bpymorse

class Mesh(AbstractComponent):
    mesh_primitive_add = bpymorse.add_mesh_monkey # you must set it in subclasses
    def __init__(self, name):
        AbstractComponent.__init__(self)
        bpymorse.deselect_all()
        self.mesh_primitive_add()
        obj = bpymorse.get_first_selected_object()
        obj.name = name
        # no collision by default for components
        obj.game.physics_type = 'NO_COLLISION'
        self.set_blender_object(obj)

    def color(self, r=0.1, g=0.1, b=0.1):
        if not self._bpy_object.active_material:
            self._bpy_object.active_material = bpymorse.create_new_material()
        self._bpy_object.active_material.diffuse_color = (r, g, b)

class Plane(Mesh):
    mesh_primitive_add = bpymorse.add_mesh_plane

class Cube(Mesh):
    mesh_primitive_add = bpymorse.add_mesh_cube

class Sphere(Mesh):
    mesh_primitive_add = bpymorse.add_mesh_uv_sphere

class IcoSphere(Mesh):
    mesh_primitive_add = bpymorse.add_mesh_ico_sphere

class Cylinder(Mesh):
    mesh_primitive_add = bpymorse.add_mesh_cylinder

class Cone(Mesh):
    mesh_primitive_add = bpymorse.add_mesh_cone

class Torus(Mesh):
    mesh_primitive_add = bpymorse.add_mesh_torus

class Spot(AbstractComponent):
    def __init__(self, name, lamp_type='SPOT'):
        AbstractComponent.__init__(self)
        bpymorse.deselect_all()
        bpymorse.add_lamp(type=lamp_type)
        obj = bpymorse.get_first_selected_object()
        obj.name = name
        # no collision by default for components
        obj.game.physics_type = 'NO_COLLISION'
        self.set_blender_object(obj)
        # Emit in +X
        self.rotate(y=-math.pi/2)
        if lamp_type is 'SPOT':
            spot = bpymorse.get_last_lamp()
            spot.spot_size = math.pi / 2
            spot.distance = 10

class Camera(AbstractComponent):
    def __init__(self, name):
        AbstractComponent.__init__(self)
        bpymorse.deselect_all()
        bpymorse.add_camera()
        obj = bpymorse.get_first_selected_object()
        obj.name = name
        # no collision by default for components
        obj.game.physics_type = 'NO_COLLISION'
        self.set_blender_object(obj)
        # Camera look in +Z
        self.rotate(y=math.pi)
