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

"""
# following is 2.57 (bpy.data.libraries.load)
filepath = '/usr/local/share/data/morse/components/robots/atrv.blend'
with bpy.data.libraries.load(filepath) as (src, dest):
  for obj in src.objects:
    tmp = obj
    suffix = 1
    while tmp in bpy.data.objects.keys():
      tmp='%s%03d' % (obj, suffix)
      suffix = suffix + 1
    dest.objects[tmp] = src.objects[obj]

for obj in dest.objects:
  bpy.context.scene.objects.link(bpy.data.objects[obj])

bpy.ops.object.constraints_clear() # not sure about that, but update the parent

# it allow us to avoid the components-dictionnary
# to do so, we have to give specific name to the main-object
# ie: for atrv.blend, ATRV -> morseMainATRV
# ...still no way to get back the 'imported' name...
# http://www.blender.org/documentation/250PythonDoc/bpy.types.BlendDataLibraries.html#bpy.types.BlendDataLibraries.load
def poc(category, name):
  objmain = None
  filepath = os.path.join(MORSE_COMPONENTS, category, name + '.blend')
  with bpy.data.libraries.load(filepath) as (src, dest):
    dest.objects = src.objects
  for obj in dest.objects:
    bpy.context.scene.objects.link(bpy.data.objects[obj])
    if obj.startswith('morseMain'):
      objmain = obj
  return objmain
"""
class Component(object):
  """ Append a morse-component to the scene
  http://www.blender.org/documentation/250PythonDoc/bpy.ops.wm.html#bpy.ops.wm.link_append
  """
  _config = Configuration()
  def __init__(self, category, name):
    objlist = [{'name':obj} for obj in MORSE_COMPONENTS_DICT[category][name]]
    objname = MORSE_COMPONENTS_DICT[category][name][0] # name of the main object
    objpath = os.path.join(MORSE_COMPONENTS, category, name + '.blend/Object/')
    # if there is already an object 'X' in the scene, Blender changes its name 
    # in 'X.001', but link_append returns just {'FINISHED'} or {'CANCELLED'},
    # so here we redo what Blender does... (I know, not pretty)
    tmp = objname
    suffix = 1 # ugly, but fix the bug
    while objname in bpy.data.objects.keys():
      objname='%s.%03d' % (tmp, suffix)
      suffix = suffix + 1
    bpy.ops.wm.link_append(directory=objpath, link=False, files=objlist)
    # is there a way to get back the object, or its imported name, once we did?
    self._blendname = name # for middleware dictionary
    self._blendobj = bpy.data.objects[objname]
  def append(self, obj):
    """ Add a child to the current object,
    eg: robot.append(sensor), will set the robot parent of the sensor.
    cf: bpy.ops.object.parent_set()
    """
    opsobj = bpy.ops.object
    opsobj.select_all(action = 'DESELECT')
    opsobj.select_name(name = obj.name)
    bpy.ops.object.make_local()
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
    """
    prop = self._blendobj.game.properties
    for k in kwargs.keys():
      prop[k].value = kwargs[k]

# self._blendobj.scale
# self._blendobj.parent
# self._blendobj.children

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

