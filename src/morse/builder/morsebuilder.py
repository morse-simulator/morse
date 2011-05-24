import os
import bpy
from morse.builder.data import *

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

class Component(object):
  """ Append a morse-component to the scene
  http://www.blender.org/documentation/blender_python_api_2_57_release/bpy.ops.wm.html#bpy.ops.wm.link_append
  """
  _config = Configuration()
  def __init__(self, category, name):
    objlist = [{'name':obj} for obj in MORSE_COMPONENTS_DICT[category][name]]
    objpath = os.path.join(MORSE_COMPONENTS, category, name + '.blend/Object/')
    bpy.ops.object.select_all(action='DESELECT')
    bpy.ops.wm.link_append(directory=objpath, link=False, files=objlist)
    # is there a way to get back the object, or its imported name, once we did?
    self._blendname = name # for middleware dictionary
    self._blendobj = bpy.context.selected_objects[0]
  def append(self, obj):
    """ Add a child to the current object,
    eg: robot.append(sensor), will set the robot parent of the sensor.
    cf: bpy.ops.object.parent_set()
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
  def location(self, x=0.0, y=0.0, z=0.0):
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
    """ modify the game properties of the Blender object
    http://www.blender.org/documentation/blender_python_api_2_57_release/bpy.types.Object.html#bpy.types.Object.game
    http://www.blender.org/documentation/blender_python_api_2_57_release/bpy.types.GameObjectSettings.html#bpy.types.GameObjectSettings.properties
    http://www.blender.org/documentation/blender_python_api_2_57_release/bpy.types.GameProperty.html#bpy.types.GameProperty
    """
    prop = self._blendobj.game.properties
    for k in kwargs.keys():
      prop[k].value = kwargs[k]


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

