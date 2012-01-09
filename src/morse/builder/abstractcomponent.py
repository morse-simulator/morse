import logging; logger = logging.getLogger("morsebuilder." + __name__)
import bpy
import json

from morse.builder.data import *

# TODO check if scene_modifiers == AbstractComponent._config.modifier.keys() ?
scene_modifiers = []

class Configuration(object):
    """ class morse.builder.Configuration

    meant to be singleton (one static instance shared by all components)
    contains the configuration of the simulation.
    """
    def __init__(self):
        self.middleware = {}
        self.modifier = {}
        self.service = {}
        self.overlay = {}

    def link_mw(self, component, mw_method_cfg):
        self.middleware.setdefault(component.name, []).append(mw_method_cfg)

    def link_service(self, component, service_cfg):
        # Special case here for the pseudo component 'simulation' that
        # covers the simulator management services.
        if component == "simulation":
            self.service.setdefault(component, []).append(service_cfg)
        else:
            self.service.setdefault(component.name, []).append(service_cfg)

    def link_modifier(self, component, modifier_cfg):
        self.modifier.setdefault(component.name, []).append(modifier_cfg)

    def link_overlay(self, component,  manager, overlay_cfg):
        self.overlay.setdefault(manager, {})[component.name] = overlay_cfg 

    def write_config(self):
        """ Write the 'component_config.py' file with the supplied settings """
        if not 'component_config.py' in bpy.data.texts.keys():
            bpy.ops.text.new()
            bpy.data.texts[-1].name = 'component_config.py'
        cfg = bpy.data.texts['component_config.py']
        cfg.clear()
        cfg.write('component_mw = ' + json.dumps(self.middleware, indent=1) )
        cfg.write('\n')
        cfg.write('component_modifier = ' + json.dumps(self.modifier, indent=1) )
        cfg.write('\n')
        cfg.write('component_service = ' + json.dumps(self.service, indent=1) )
        cfg.write('\n')
        cfg.write('overlays = ' + json.dumps(self.overlay, indent=1) )
        cfg.write('\n')

class AbstractComponent(object):
    # static config common to all component of the simulation
    _config = Configuration()
    def __init__(self, obj=None, name=None):
        self._blendobj = obj
        self._blendname = name # for mw config
    def append(self, obj):
        """ Add a child to the current object,

        eg: robot.append(sensor), will set the robot parent of the sensor.
        """
        obj._blendobj.parent = self._blendobj

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
    def property_value(name):
        return self._blendobj.game.properties[name].value
    def properties(self, **kwargs):
        """ Add/modify the game properties of the Blender object

        Usage example:

        .. code-block:: python
            self.properties(Component_Tag = True, Class='XXXClass', speed = 5.0)

        will create and/or set the 3 game properties Component_Tag, Class, and 
        speed at the value True (boolean), XXXClass (string), 5.0 (float).
        In Python the type of numeric value is 'int', if you want to force it to
        float, use the following: float(5) or 5.0
        Same if you want to force to integer, use: int(a/b)
        For the TIMER type, see the class timer(float) defined in this module:

        .. code-block:: python
            self.properties(My_Clock = timer(5.0), My_Speed = int(5/2))

        """
        prop = self._blendobj.game.properties
        for k in kwargs.keys():
            if k in prop.keys():
                prop[k].value = kwargs[k]
            else:
                self._property_new(k, kwargs[k])

    def _property_new(self, n, v, t=None):
        """ Add a new game property for the Blender object

        :param n: property name (string)
        :param v: property value
        :param t: property type (enum in ['BOOL', 'INT', 'FLOAT', 'STRING', 'TIMER'], 
                optional, auto-detect, default=None)
        """
        bpy.ops.object.select_all(action = 'DESELECT')
        bpy.ops.object.select_name(name = self.name)
        bpy.ops.object.game_property_new()
        prop = self._blendobj.game.properties
        # select the last property in the list (which is the one we just added)
        prop[-1].name = n
        if t == None:
            # Detect the type (class name upper case)
            t = v.__class__.__name__.upper()
        if t == 'STR':
            # Blender property string are called 'STRING' (and not 'str' as in Python)
            t = 'STRING'
        prop[-1].type = t
        prop[-1].value = v

    def configure_mw(self, mw, config=None, method=None, path=None, component=None):
        """

        :param component: if set, force to use the configuration of the given 
        component, instead of our own (default=None).
        """
        # Configure the middleware for this component
        if not config:
            if not method:
                try:
                    if component:
                        config = MORSE_MIDDLEWARE_DICT[mw][component]
                    else:
                        # TODO self._blendobj.game.properties["Class"].value ?
                        #      as map-key (in data, instead of blender-filename)
                        config = MORSE_MIDDLEWARE_DICT[mw][self._blendname]
                except KeyError:
                    logger.warning("%s: default middleware method"%self.name)
                    # set the default method either it is an Actuator or a Sensor
                    method = 'XXX_message'
                    prop = self._blendobj.game.properties
                    if 'Path' in prop:
                        if prop['Path'].value.startswith('morse/sensors/'):
                            method = 'post_message'
                        elif prop['Path'].value.startswith('morse/actuators/'):
                            method = 'read_message'
                    config = [MORSE_MIDDLEWARE_MODULE[mw], method]
            else:
                if not path:
                    config = [MORSE_MIDDLEWARE_MODULE[mw], method]
                else:
                    config = [MORSE_MIDDLEWARE_MODULE[mw], method, path]
        AbstractComponent._config.link_mw(self, config)

    def configure_service(self, mw, component = None):
        if not component:
            component = self
        service = MORSE_SERVICE_DICT[mw]
        AbstractComponent._config.link_service(component, service)

    def configure_modifier(self, mod, config=None):
        # Configure the middleware for this component
        if not config:
            config = MORSE_MODIFIER_DICT[mod][self._blendname]
        AbstractComponent._config.link_modifier(self, config)
        
    def configure_overlay(self, mw, overlay):
        request_manager = MORSE_SERVICE_DICT[mw]
        AbstractComponent._config.link_overlay(self, request_manager, overlay)

class timer(float):
    __doc__ = "this class extends float for the game properties configuration"
