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

cfg_middleware = {}
cfg_modifier = {}
cfg_service = {}
cfg_overlay = {}

class Configuration(object):
    def __init__(self):
        if not 'component_config.py' in bpy.data.texts.keys():
            bpy.ops.text.new()
            bpy.data.texts[-1].name = 'component_config.py'

    def link_mw(self, component, mw_method_cfg):
        cfg_middleware.setdefault(component.name, []).append(mw_method_cfg)

    def link_service(self, component, service_cfg):
        cfg_service.setdefault(component.name, []).append(service_cfg)

    def link_modifier(self, component, modifier_cfg):
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

class timer(float):
    __doc__ = "this class extends float for the game properties configuration"

class Component(AbstractComponent):
    """ Append a morse-component to the scene

    cf. `bpy.ops.wm.link_append 
    <http://www.blender.org/documentation/blender_python_api_2_57_release/bpy.ops.wm.html#bpy.ops.wm.link_append>`_ 
     and 
    `bpy.data.libraries.load 
    <http://www.blender.org/documentation/blender_python_api_2_57_release/bpy.types.BlendDataLibraries.html>`_ 
    """
    def __init__(self, category, name):
        AbstractComponent.__init__(self)
        filepath = os.path.join(MORSE_COMPONENTS, category, name + '.blend')

        if bpy.app.version > (2,56,0):
            with bpy.data.libraries.load(filepath) as (src, _):
                try:
                    objlist = [{'name':obj} for obj in src.objects]
                except UnicodeDecodeError as detail:
                    print ("Unable to open file '%s'. Exception: %s" % (filepath, detail))
        else: # Blender 2.56 does not support bpy.data.libraries.load
            objlist = [{'name':obj} for obj in MORSE_COMPONENTS_DICT[category][name]]

        #print ("NAME: %s | CATEGORY: %s | objlist %s" % (name, category, objlist))

        if category == 'middleware' or category == 'modifiers':
                # Avoid re-inserting middleware or modifier objects
                if objlist[0]['name'] in bpy.data.objects:
                        self._blendname = name
                        return
                else:
                        print ("Adding Empty component '%s'" % name)

        bpy.ops.object.select_all(action='DESELECT')
        bpy.ops.wm.link_append(directory=filepath + '/Object/', link=False, 
                autoselect=True, files=objlist)
        self._blendname = name # for middleware dictionary
        # here we use the fact that after appending, Blender select the objects 
        # and the root (parent) object first ( [0] )
        self._blendobj = bpy.context.selected_objects[0]

    def configure_mw(self, mw, config=None):
        # Add the middleware empty objects as needed
        mw_name = mw + "_mw"
        Middleware(mw_name)
        if not config:
            config = MORSE_MIDDLEWARE_DICT[mw][self._blendname]
        Component._config.link_mw(self, config)
        #Component._config.write()

    def configure_service(self, service):
        Component._config.link_service(self, service)

    def configure_modifier(self, mod):
        # Add the modifier empty objects as needed
        mod_name = mod[0].lower()
        Modifier(mod_name)
        config = mod
        Component._config.link_modifier(self, config)

    def configure_overlay(self, manager, overlay):
        o = self._blendobj
        klass = o.game.properties["Class"].value
        Component._config.link_overlay(klass, manager, overlay)


class Robot(Component):
    def __init__(self, name):
        Component.__init__(self, 'robots', name)

class Sensor(Component):
    def __init__(self, name):
        Component.__init__(self, 'sensors', name)

class Actuator(Component):
    def __init__(self, name):
        Component.__init__(self, 'actuators', name)

class Middleware(Component):
    def __init__(self, name):
        Component.__init__(self, 'middleware', name)

class Modifier(Component):
    def __init__(self, name):
        Component.__init__(self, 'modifiers', name)

class Environment(Component):
    def __init__(self, name):
        Component.__init__(self, 'environments', name)
        self._camera_location = [5, -5, 5]
        self._camera_rotation = [0.7854, 0, 0.7854]

    def _write(self):
        cfg = bpy.data.texts['component_config.py']
        cfg.clear()
        cfg.write('component_mw = ' + json.dumps(cfg_middleware, indent=1) )
        cfg.write('\n')
        cfg.write('component_modifier = ' + json.dumps(cfg_modifier, indent=1) )
        cfg.write('\n')
        cfg.write('component_service = ' + json.dumps(cfg_service, indent=1) )
        cfg.write('\n')
        cfg.write('overlays = ' + json.dumps(cfg_overlay, indent=1) )
        cfg.write('\n')

    def _get_path_file(self):
        """ Get the file which should be in the 'props/basics.blend' file """
        filepath = os.path.join(MORSE_COMPONENTS, 'props/basics.blend')
        with bpy.data.libraries.load(filepath) as (src, dest):
                print ("ESTO: ", src.texts[0].as_string())
                dest.texts['setup_path.py'] = src.texts['setup_path.py']

    def place_camera(self, location):
        """ Store the position that will be givent to the default camera
        Expected argument is a list with the desired position for the camera """
        self._camera_location = location

    def aim_camera(self, rotation):
        """ Store the orientation that will be givent to the default camera
        Expected argument is a list with the desired orientation for the camera """
        self._camera_rotation = rotation

    def __del__(self):
        """ Generate the scene configuration and insert necessary objects
        """
        # Write the configuration of the middlewares
        self._write()
        # Add the necessary objects
        base = Component('props', 'basics')
        # Set the position of the camera
        camera_fp = bpy.data.objects['CameraFP']
        camera_fp.location = self._camera_location
        camera_fp.rotation_euler = self._camera_rotation
        # Make CameraFP the active camera
        bpy.ops.object.select_all(action = 'DESELECT')
        bpy.ops.object.select_name(name = 'CameraFP')
