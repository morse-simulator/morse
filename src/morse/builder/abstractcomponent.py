import os
import bpy
import json
#import logging; logger = logging.getLogger("morse." + __name__)
#logger.setLevel(logging.DEBUG)

from morse.builder.data import *

cfg_middleware = {}
cfg_modifier = {}
cfg_service = {}
cfg_overlay = {}
scene_modifiers = []

class Configuration(object):
    def link_mw(self, component, mw_method_cfg):
        cfg_middleware.setdefault(component.name, []).append(mw_method_cfg)

    def link_service(self, component, service_cfg):
        cfg_service.setdefault(component.name, []).append(service_cfg)

    def link_modifier(self, component, modifier_cfg):
        mod_class = MORSE_MODIFIERS[modifier_cfg[0]]
        modifier_cfg[0] = mod_class
        cfg_modifier.setdefault(component.name, []).append(modifier_cfg)

    def link_overlay(self, klass,  manager, overlay_cfg):
        cfg_overlay.setdefault(manager, {})[klass] = overlay_cfg 

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
        obj._blendobj.parent = self._blendobj

        #opsobj = bpy.ops.object
        #opsobj.select_all(action = 'DESELECT')
        #opsobj.select_name(name = obj.name)
        #opsobj.make_local()
        #opsobj.select_name(name = self.name)
        #opsobj.parent_set()

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
        <http://www.blender.org/documentation/blender_python_api_2_57_release/bpy.types.Object.html#bpy.types.Object.location>`_ 
        """
        old = self._blendobj.location
        self._blendobj.location = (old[0]+x, old[1]+y, old[2]+z)
    def rotate(self, x=0.0, y=0.0, z=0.0):
        """ Rotation in Eulers, float array of 3 items in [-inf, inf], 
        default (0.0, 0.0, 0.0)

        cf. `bpy.types.Object.rotation_euler 
        <http://www.blender.org/documentation/blender_python_api_2_57_release/bpy.types.Object.html#bpy.types.Object.rotation_euler>`_ (x*math.pi/180)
        """
        old = self._blendobj.rotation_euler
        self._blendobj.rotation_euler = (old[0]+x, old[1]+y, old[2]+z)
    def properties(self, **kwargs):
        """ Add/modify the game properties of the Blender object

        `bpy.types.Object.game 
        <http://www.blender.org/documentation/blender_python_api_2_57_release/bpy.types.Object.html#bpy.types.Object.game>`_
        `bpy.types.GameObjectSettings.properties 
        <http://www.blender.org/documentation/blender_python_api_2_57_release/bpy.types.GameObjectSettings.html#bpy.types.GameObjectSettings.properties>`_
        `bpy.types.GameProperty 
        <http://www.blender.org/documentation/blender_python_api_2_57_release/bpy.types.GameProperty.html#bpy.types.GameProperty>`_
        """
        prop = self._blendobj.game.properties
        for k in kwargs.keys():
            if k in prop.keys():
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
        if t == None:
            t = v.__class__.__name__.upper()
        if t == 'STR':
            t = 'STRING'
        o.game.properties[n].type = t
        o.game.properties[n].value = v

    def configure_mw(self, mw, config=None):
        # Configure the middleware for this component
        if not config:
            config = MORSE_MIDDLEWARE_DICT[mw][self._blendname]
        AbstractComponent._config.link_mw(self, config)

    def configure_service(self, mw):
        service = MORSE_SERVICE_DICT[mw]
        AbstractComponent._config.link_service(self, service)

    def configure_modifier(self, mod):
        # Configure the middleware for this component
        config = mod
        AbstractComponent._config.link_modifier(self, config)
        
    def configure_overlay(self, manager, overlay):
        o = self._blendobj
        klass = o.game.properties["Class"].value
        AbstractComponent._config.link_overlay(klass, manager, overlay)

class timer(float):
    __doc__ = "this class extends float for the game properties configuration"
