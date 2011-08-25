import os
import bpy
import json
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
    if not 'component_config.py' in bpy.data.texts:
      bpy.ops.text.new()
      bpy.data.texts[-1].name = 'component_config.py'
      # make the current scene Morse-able
      if not 'Scene_Script_Holder' in bpy.data.objects:
        filepath = os.path.join(os.environ["MORSE_ROOT"], "share", "data", 
            "morse", "morse_default.blend")
        objlist = [{'name': 'CameraFP'}, {'name': 'HUD_plane'}, 
            {'name': 'Title_text'}, {'name': 'Keys_text'}, {'name': 'Compass'}, 
            {'name': 'Scene_Script_Holder'}]
        bpy.ops.wm.link_append(directory=filepath + '/Text/', link=False, 
            files=[{'name': 'setup_path.py'}])
        bpy.ops.wm.link_append(directory=filepath + '/Object/', link=False, 
            files=objlist)
  def write(self):
    cfg = bpy.data.texts['component_config.py']
    cfg.clear()
    cfg.write('component_mw = ' + json.dumps(self.middleware, indent=1) )
    cfg.write('\n')
    cfg.write('component_modifier = ' + json.dumps(self.modifier, indent=1) )
    cfg.write('\n')
    cfg.write('component_service = ' + json.dumps(self.service, indent=1) )
    cfg.write('\n')
  def link(self, component, mwmethodcfg):
    self.middleware[component.name] = mwmethodcfg


class AbstractComponent(object):
  # static config common to all component of the simulation
  _config = Configuration()
  def __init__(self):
    self._blendobj = None
    self._blendname = None # for middleware configuration
  def append(self, obj):
    """ Add a child (obj) to the current component,

    obj.name must exists (can be either a blender object or a Morse Component)
    eg: robot.append(sensor), will set the robot parent of the sensor.
    `bpy.ops.object.parent_set() 
    <http://www.blender.org/documentation/blender_python_api_2_58_release/bpy.ops.object.html#bpy.ops.object.parent_set>`_ 
    """
    opsobj = bpy.ops.object
    # make sure that nothing is selected
    opsobj.select_all(action = 'DESELECT')
    # select the futur child
    opsobj.select_name(name = obj.name)
    opsobj.make_local()
    # select the parent
    opsobj.select_name(name = self.name, extend=True)
    # establish the relationship
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
  def translate(self, x=0.0, y=0.0, z=0.0):
    """ Location of the object, float array of 3 items in [-inf, inf], 
    default (0.0, 0.0, 0.0)

    cf. `bpy.types.Object.location 
    <http://www.blender.org/documentation/blender_python_api_2_58_release/bpy.types.Object.html#bpy.types.Object.location>`_ 
    """
    old = self._blendobj.location
    self._blendobj.location = (old.x + x, old.y + y, old.z + z)
  def rotate(self, x=0.0, y=0.0, z=0.0):
    """ Rotation in Eulers, float array of 3 items in [-inf, inf], 
    default (0.0, 0.0, 0.0)

    cf. `bpy.types.Object.rotation_euler 
    <http://www.blender.org/documentation/blender_python_api_2_58_release/bpy.types.Object.html#bpy.types.Object.rotation_euler>`_ (x*math.pi/180)
    """
    old = self._blendobj.rotation_euler
    self._blendobj.rotation_euler = (old.x + x, old.y + y, old.z + z)
  def properties(self, **kwargs):
    """ Add/modify the game properties of the Blender object

    `bpy.types.Object.game 
    <http://www.blender.org/documentation/blender_python_api_2_58_release/bpy.types.Object.html#bpy.types.Object.game>`_
    `bpy.types.GameObjectSettings.properties 
    <http://www.blender.org/documentation/blender_python_api_2_58_release/bpy.types.GameObjectSettings.html#bpy.types.GameObjectSettings.properties>`_
    `bpy.types.GameProperty 
    <http://www.blender.org/documentation/blender_python_api_2_58_release/bpy.types.GameProperty.html#bpy.types.GameProperty>`_
    """
    prop = self._blendobj.game.properties
    for k in kwargs:
      if k in prop:
        prop[k].value = kwargs[k]
      else:
        self._property_new(k, kwargs[k])

  def _property_new(self, n, v, t=None):
    """ Add a new game property for the Blender object

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
    if not t:
      t = v.__class__.__name__.upper()
    if t == 'STR':
      t = 'STRING'
    o.game.properties[n].type = t
    o.game.properties[n].value = v

class timer(float):
  __doc__ = "this class extends float for the game properties configuration"

class Component(AbstractComponent):
  """ Append a morse-component to the scene

  cf. `bpy.ops.wm.link_append 
  <http://www.blender.org/documentation/blender_python_api_2_58_release/bpy.ops.wm.html#bpy.ops.wm.link_append>`_ 
   and 
  `bpy.data.libraries.load 
  <http://www.blender.org/documentation/blender_python_api_2_58_release/bpy.types.BlendDataLibraries.html>`_ 
  """
  def __init__(self, category, name):
    AbstractComponent.__init__(self)
    filepath = os.path.join(MORSE_COMPONENTS, category, name + '.blend')

    with bpy.data.libraries.load(filepath) as (src, _):
        objlist = [{'name':obj} for obj in src.objects]

    #print ("NAME: %s | CATEGORY: %s | objlist %s" % (name, category, objlist))

    if category == 'middleware' and objlist[0]['name'] in bpy.data.objects:
        #print ("Middleware '%s' is already in the scene" % name)
        self._blendname = name
        return

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
  def configure(self, component, config=None):
    """ Component bindings with middlewares (hooks)

    http://www.openrobots.org/morse/doc/latest/user/hooks.html#configuration
    """
    if not config:
      config = MORSE_MIDDLEWARE_DICT[self._blendname][component._blendname]
    Component._config.link(component, config)
    Component._config.write()

