import bpy
from morse.builder.morsebuilder import *

###############################################################################
# following is experimental:
#
class MotionControler(AbstractComponent):
  """ Add MotionControler 100% Python

  http://www.blender.org/documentation/blender_python_api_2_57_release/bpy.types.Controller.html#bpy.types.Controller.link
  http://www.blender.org/documentation/blender_python_api_2_57_release/bpy.ops.logic.html#bpy.ops.logic.sensor_add
  http://www.blender.org/documentation/blender_python_api_2_57_release/bpy.ops.logic.html#bpy.ops.logic.controller_add
  """
  def __init__(self):
    AbstractComponent.__init__(self)
    bpy.ops.object.select_all(action = 'DESELECT')
    bpy.ops.object.add() # default is empty
    self._blendobj = bpy.context.selected_objects[0]
    self._blendobj.name = "Motion_Controller"
    self._blendname = "morse_vw_control" # for mw config
    self.properties(Component_Tag = True, Class = "VWActuatorClass", 
        Path = "morse/actuators/v_omega")
    bpy.ops.object.select_all(action = 'DESELECT')
    bpy.ops.object.select_name(name = self._blendobj.name)
    bpy.ops.logic.sensor_add()
    sensor = self._blendobj.game.sensors.keys()[-1]
    self._blendobj.game.sensors[sensor].use_pulse_true_level = True
    bpy.ops.logic.controller_add(type='PYTHON')
    controller = self._blendobj.game.controllers.keys()[-1]
    self._blendobj.game.controllers[controller].mode = 'MODULE'
    self._blendobj.game.controllers[controller].module = 'calling.actuator_action'
    self._blendobj.game.controllers[controller].link( sensor = 
        self._blendobj.game.sensors[sensor] )

class BlenderComponent(AbstractComponent):
  """ Add a Blender object to the scene

  object_name (string, (optional))
  object_type (enum in ['MESH', 'CURVE', 'SURFACE', 'META', 'FONT', 'ARMATURE', 'LATTICE', 'EMPTY', 'CAMERA', 'LAMP'], (optional))
  http://www.blender.org/documentation/blender_python_api_2_57_release/bpy.ops.object.html#bpy.ops.object.add
  TODO
  http://www.blender.org/documentation/blender_python_api_2_57_release/bpy.ops.object.html#bpy.ops.object.lamp_add
  http://www.blender.org/documentation/blender_python_api_2_57_release/bpy.ops.mesh.html#bpy.ops.mesh.primitive_plane_add
  ...
  """
  def __init__(self, object_type='EMPTY', object_name=None, 
      object_location=(0.0, 0.0, 0.0), object_rotation=(0.0, 0.0, 0.0)):
    Component.__init__(self)
    bpy.ops.object.select_all(action = 'DESELECT')
    bpy.ops.object.add(type = object_type, location=object_location, 
        rotation=object_rotation)
    self._blendobj = bpy.context.selected_objects[0]
    if object_name != None:
      self._blendobj.name = object_name

class ATRV(AbstractComponent):
  """ Add ATRV robot from Collada and add Game Engine properties, sensor and
  controler using Blender Python API (bpy)
  """
  def __init__(self):
    AbstractComponent.__init__(self)
    bpy.ops.object.select_all(action = 'DESELECT')
    bpy.ops.wm.collada_import(filepath="/usr/local/share/data/morse/components/robots/atrv.dae")
    tmp=bpy.context.selected_objects[0]
    # collada_import only select the last imported object
    # which can be a wheel
    while tmp.parent != None:
      tmp = tmp.parent
    self._blendobj = tmp
    self._blendobj.name = "ATRV"
    self.properties(Robot_Tag = True, Class = "ATRVClass", 
        Path = "morse/robots/atrv")
    bpy.ops.logic.sensor_add()
    sensor = self._blendobj.game.sensors.keys()[-1]
    self._blendobj.game.sensors[sensor].use_pulse_true_level = True
    bpy.ops.logic.controller_add(type='PYTHON')
    controller = self._blendobj.game.controllers.keys()[-1]
    self._blendobj.game.controllers[controller].mode = 'MODULE'
    self._blendobj.game.controllers[controller].module = 'calling.robot_action'
    self._blendobj.game.controllers[controller].link( sensor = 
        self._blendobj.game.sensors[sensor] )
    # Physic properties
    self._blendobj.game.physics_type = 'RIGID_BODY'
    self._blendobj.game.use_sleep = True
    self._blendobj.game.mass = 20.0
    self._blendobj.game.radius = .5
    self._blendobj.game.damping = .025
    self._blendobj.game.rotation_damping = .180
    self._blendobj.game.use_collision_bounds = True
    self._blendobj.game.collision_bounds_type = 'CONVEX_HULL'
    self._blendobj.game.collision_margin = .05

def print_prop(obj):
  for p in obj.game.properties.keys():
    print("%s = %s, "%(p,obj.game.properties[p].value))

def delete_all():
  bpy.ops.object.select_all(action='SELECT')
  bpy.ops.object.delete()

def import_scene(filepath = "/usr/local/share/data/morse/morse_default.blend"):
  delete_all()
  if bpy.app.version[1] > 56:
    with bpy.data.libraries.load(filepath) as (src, _):
      objlist = [{'name':obj} for obj in src.objects]
  else: # Blender 2.56 does not support bpy.data.libraries.load
    objlist = [{'name': 'whiteboard'}, {'name': 'Shelf_3'}, {'name': 'Shelf_2'}, {'name': 'Shelf_1'}, {'name': 'Scene_Script_Holder'}, {'name': 'Rolling_Chair'}, {'name': 'red_heat.001'}, {'name': 'red_heat'}, {'name': 'Plane.001'}, {'name': 'Plane'}, {'name': 'Lamp.003'}, {'name': 'Lamp.002'}, {'name': 'Lamp.001'}, {'name': 'Lamp'}, {'name': 'Ground'}, {'name': 'Desk_3'}, {'name': 'Desk_2'}, {'name': 'Desk_1'}, {'name': 'Compass'}, {'name': 'CameraFP'}]
  bpy.ops.wm.link_append(directory=filepath + '/Object/', link=False, 
        files=objlist)
  # TODO import as well: mesh / textures / etc...


