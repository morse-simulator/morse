import os
import bpy
from morse.builder.data import *

"""
Morse Builder API

To test this module you can c/p the following code in Blender Python console::

  import sys
  sys.path.append("/usr/local/lib/python3.1/dist-packages")
  from morse.builder.morsebuilder import *
  atrv=Robot("atrv")

The string passed to the differents Components Classes must be an existing 
.blend file-name, ie. for ``Robot("atrv")`` the file ``atrv.blend`` must exists 
in the folder ``MORSE_COMPONENTS/robots/``.
"""

class Configuration(object):
  def __init__(self):
    self.middleware = {}
    self.modifier = {}
    self.service = {}
  def write(self):
    cfg = bpy.data.texts['component_config.py']
    cfg.clear()
    cfg.write('component_mw = ' + str(self.middleware) )
    cfg.write('\n')
    cfg.write('component_modifier = ' + str(self.modifier) )
    cfg.write('\n')
    cfg.write('component_service = ' + str(self.service) )
    cfg.write('\n')
  def link(self, component, mwmethodcfg):
    self.middleware[component.name] = mwmethodcfg


class AbstractComponent(object):
  # static config common to all component of the simulation
  _config = Configuration() 
  def __init__(self):
    self._blendobj = None
    self._blendname = None # for mw config
  def append(self, obj):
    """ Add a child to the current object,

    eg: robot.append(sensor), will set the robot parent of the sensor.
    cf: bpy.ops.object.parent_set()
    obj._blendobj.parent = self._blendobj
    self._blendobj.children += obj._blendobj
    """
    opsobj = bpy.ops.object
    opsobj.select_all(action = 'DESELECT')
    opsobj.select_name(name = obj.name)
    opsobj.make_local()
    opsobj.select_name(name = self.name)
    opsobj.parent_set()
  @property
  def name(self):
    return self._blendobj.name
  @name.setter
  def name(self, value):
    self._blendobj.name = value
  @property
  def location(self):
    return self._blendobj.location
  @location.setter
  def location(self, xyz):
    self._blendobj.location = xyz
  @property
  def scale(self):
    return self._blendobj.scale
  @scale.setter
  def scale(self, xyz):
    self._blendobj.scale = xyz
  @property
  def rotation_euler(self):
    return self._blendobj.rotation_euler
  @rotation_euler.setter
  def rotation_euler(self, xyz):
    self._blendobj.rotation_euler = xyz
  def position(self, x=0.0, y=0.0, z=0.0):
    self._blendobj.location = (x,y,z)
  def translate(self, x=0.0, y=0.0, z=0.0):
    """ cf. Object.location

    http://www.blender.org/documentation/blender_python_api_2_57_release/bpy.types.Object.html#bpy.types.Object.location
    Location of the object, float array of 3 items in [-inf, inf], default (0.0, 0.0, 0.0)
    """
    old = self._blendobj.location
    self._blendobj.location = (old[0]+x, old[1]+y, old[2]+z)
  def rotate(self, x=0.0, y=0.0, z=0.0):
    """ cf. Object.rotation_euler (x*math.pi/180)

    http://www.blender.org/documentation/blender_python_api_2_57_release/bpy.types.Object.html#bpy.types.Object.rotation_euler
    Rotation in Eulers, float array of 3 items in [-inf, inf], default (0.0, 0.0, 0.0)
    """
    old = self._blendobj.rotation_euler
    self._blendobj.rotation_euler = (old[0]+x, old[1]+y, old[2]+z)
  def properties(self, **kwargs):
    """ add/modify the game properties of the Blender object

    http://www.blender.org/documentation/blender_python_api_2_57_release/bpy.types.Object.html#bpy.types.Object.game
    http://www.blender.org/documentation/blender_python_api_2_57_release/bpy.types.GameObjectSettings.html#bpy.types.GameObjectSettings.properties
    http://www.blender.org/documentation/blender_python_api_2_57_release/bpy.types.GameProperty.html#bpy.types.GameProperty
    """
    prop = self._blendobj.game.properties
    for k in kwargs.keys():
      if k in prop.keys():
        prop[k].value = kwargs[k]
      else:
        self.property_new(k, kwargs[k])

  def property_new(self, n, v, t=None):
    """ add a new game property for the Blender object

    n: property name (string)
    v: property value
    t: property type (enum in ['BOOL', 'INT', 'FLOAT', 'STRING', 'TIMER'], 
        optional, auto-detect, default=None)
    """
    o = self._blendobj
    bpy.ops.object.select_all(action = 'DESELECT')
    bpy.ops.object.select_name(name = o.name)
    bpy.ops.object.game_property_new()
    # select the last property in the list
    x = o.game.properties.keys()[-1]
    o.game.properties[x].name = n
    if t == None:
      t = v.__class__.__name__.upper()
    if t == 'STR':
      t = 'STRING'
    o.game.properties[n].type = t
    o.game.properties[n].value = v

class timer(float):
  __doc__ = "this class extends float for the game properties configuration"

class Component(AbstractComponent):
  """ Append a morse-component to the scene
  http://www.blender.org/documentation/blender_python_api_2_57_release/bpy.ops.wm.html#bpy.ops.wm.link_append
  """
  def __init__(self, category, name):
    AbstractComponent.__init__(self)
    filepath = os.path.join(MORSE_COMPONENTS, category, name + '.blend')
    if bpy.app.version[1] > 56:
      with bpy.data.libraries.load(filepath) as (src, _):
        objlist = [{'name':obj} for obj in src.objects]
    else: # Blender 2.56 does not support bpy.data.libraries.load
      objlist = [{'name':obj} for obj in MORSE_COMPONENTS_DICT[category][name]]
    bpy.ops.object.select_all(action='DESELECT')
    bpy.ops.wm.link_append(directory=filepath + '/Object/', link=False, 
        autoselect=True, files=objlist)
    self._blendname = name # for middleware dictionary
    # here we use the fact that after appending, Blender select the objects 
    # and the root (parent) object first ( [0] )
    self._blendobj = bpy.context.selected_objects[0]

class Robot(Component):
  def __init__(self, name):
    Component.__init__(self, 'robots', name)

class Sensor(Component):
  def __init__(self, name):
    Component.__init__(self, 'sensors', name)

class Controller(Component):
  def __init__(self, name):
    Component.__init__(self, 'controllers', name)

class Middleware(Component):
  def __init__(self, name):
    Component.__init__(self, 'middleware', name)
  def configure(self, component):
    mw_config = MORSE_MIDDLEWARE_DICT[self._blendname][component._blendname]
    Component._config.link(component, mw_config)
    Component._config.write()



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
    bpy.ops.logic.sensor_add() # (type='ALWAYS', name="", object="")
    sensor = self._blendobj.game.sensors.keys()[-1]
    self._blendobj.game.sensors[sensor].use_pulse_true_level = True
    bpy.ops.logic.controller_add(type='PYTHON') # (type='LOGIC_AND', name="", object="")
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
  """ Add ATRV robot from Collada (export atrv.blend to Collada 1st)
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


