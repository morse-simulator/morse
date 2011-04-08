import os
import bpy

MORSE_COMPONENTS = '/usr/local/share/data/morse/components'

"""
would be nice to be able to generate the components map.
TODO find a way to list the objects from .blend file (*/*.blend/Object/*)

def blendobjectslist(blend):
  objects = []
  fd = open(blend, 'r')
  print(fd) # TODO list *.blend/Object/*
  fd.close()
  return objects # [{'name':'main-object-name'}, {'name':'child1-name'}, ...]

def morsecomponents(path):
  components = {}
  for category in os.listdir(path):
    pathc = os.path.join(path, category)
    if os.path.isdir(pathc):
      components[category] = {}
      for blend in os.listdir(pathc):
        pathb = os.path.join(pathc, blend)
        if os.path.isfile(pathb) & blend.endswith('.blend'):
          components[category][blend[:-6]] = blendobjectslist(pathb)
  return components

MORSE_COMPONENTS_MAP = morsecomponents(MORSE_COMPONENTS)

convention:
{
  'component-directory': {
    '.blend-file': [{'name':'main-object-name'}, {'name':'child1-name'}, ...]
  }
}

http://www.blender.org/documentation/250PythonDoc/bpy.ops.wm.html#bpy.ops.wm.link_append
"""

MORSE_COMPONENTS_MAP = {
  'robots': {
    'atrv': [{'name':'ATRV'}, {'name':'Wheel.1'}, {'name':'Wheel.2'}, 
      {'name':'Wheel.3'}, {'name':'Wheel.4'}]
  },
  'sensors': {
    'morse_gyroscope': [{'name':'Gyroscope'}, {'name':'Gyro_box'}],
    'morse_GPS': [{'name':'GPS'}, {'name':'GPS_box'}],
    'morse_odometry': [{'name':'Odometry'}, {'name':'Odometry_mesh'}]
  },
  'controllers': {
    'morse_vw_control': [{'name':'Motion_Controller'}],
    'morse_xyw_control': [{'name':'Motion_Controller'}]
  },
  'middleware': {
    'ros_empty': [{'name':'ROS_Empty'}],
    'socket_empty': [{'name':'Socket_Empty'}]
  }
}

class Component(object):
  def __init__(self, category, name):
    objlist = MORSE_COMPONENTS_MAP[category][name]
    objname = objlist[0]['name'] # name of the main object
    objpath = os.path.join(MORSE_COMPONENTS, category, name + '.blend/Object/')
    bpy.ops.wm.link_append(directory=objpath, files=objlist)
    bpy.ops.object.make_local()
    self._blendobj = bpy.data.objects[objname]
  def append(self, obj):
    """ Add a child to the current object,
    eg: robot.append(sensor), will set the robot parent of the sensor.
    cf: bpy.ops.object.parent_set()
    """
    opsobj = bpy.ops.object
    opsobj.select_all(action = 'DESELECT')
    opsobj.select_name(name = obj.name)
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
  def location(self, value):
    self._blendobj.location = value
  def __str__(self):
    return self.name

class Robot(Component):
  def __init__(self, name):
    # Call the constructor of the parent class
    super(self.__class__,self).__init__('robots', name)

class Sensor(Component):
  def __init__(self, name):
    # Call the constructor of the parent class
    super(self.__class__,self).__init__('sensors', name)

class Controller(Component):
  def __init__(self, name):
    # Call the constructor of the parent class
    super(self.__class__,self).__init__('controllers', name)

class Middleware(Component):
  def __init__(self, name):
    # Call the constructor of the parent class
    super(self.__class__,self).__init__('middleware', name)

class Config(object):
  def __init__(self):
    self.middleware = {}
    self.modifier = {}
    self.service = {}
  def init(self):
    cfg = bpy.data.texts['component_config.py']
    cfg.clear()
    cfg.write('component_mw = ' + str(self.middleware) )
    cfg.write('\n')
    cfg.write('component_modifier = ' + str(self.modifier) )
    cfg.write('\n')
    cfg.write('component_service = ' + str(self.service) )
    cfg.write('\n')
  def linkV2(self, component, mwmethod):
    self.middleware[component.name] = mwmethod.config
  def link(self, component, mwmethodcfg):
    self.middleware[component.name] = mwmethodcfg


