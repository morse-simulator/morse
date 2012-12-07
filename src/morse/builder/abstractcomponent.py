import logging; logger = logging.getLogger("morsebuilder." + __name__)
import bpy
import json

from morse.builder.data import *

class Configuration(object):
    """ class morse.builder.Configuration

    Singleton (one static instance shared by all components)
    contains the configuration of the simulation.
    """
    _instance = None
    def __new__(cls, *args, **kwargs):
        if not Configuration._instance:
            Configuration._instance = object.__new__(cls, *args, **kwargs)
        return Configuration._instance

    def __init__(self):
        if not "_init" in dir(self):
            self.datastream = {}
            self.modifier = {}
            self.service = {}
            self.overlay = {}
            self._init = True

    def link_datastream(self, component, mw_method_cfg):
        self.datastream.setdefault(component.name, []).append(mw_method_cfg)

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
        cfg.write('component_datastream = ' + json.dumps(self.datastream, indent=1) )
        cfg.write('\n')
        cfg.write('component_modifier = ' + json.dumps(self.modifier, indent=1) )
        cfg.write('\n')
        cfg.write('component_service = ' + json.dumps(self.service, indent=1) )
        cfg.write('\n')
        cfg.write('overlays = ' + json.dumps(self.overlay, indent=1) )
        cfg.write('\n')

class AbstractComponent(object):
    def __init__(self, obj=None, name=None):
        self._blendobj = obj
        if obj:
            self._blendobj.matrix_parent_inverse.identity()
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
        for key in kwargs.keys():
            if key in prop.keys():
                self._property_set(key, kwargs[key])
            else:
                self._property_new(key, kwargs[key])

    def _property_new(self, name, value, ptype=None):
        """ Add a new game property for the Blender object

        :param name: property name (string)
        :param value: property value
        :param ptype: property type (enum in ['BOOL', 'INT', 'FLOAT', 'STRING', 'TIMER'],
                      optional, auto-detect, default=None)
        """
        bpy.ops.object.select_all(action = 'DESELECT')
        self._blendobj.select = True
        bpy.context.scene.objects.active = self._blendobj
        bpy.ops.object.game_property_new()
        prop = self._blendobj.game.properties
        # select the last property in the list (which is the one we just added)
        prop[-1].name = name
        return self._property_set(-1, value, ptype)

    def _property_set(self, pid, value, ptype=None):
        """ Really set the property for the property referenced by pid

        :param pid: the index of property
        :param value: the property value
        :param ptype: property type (enum in ['BOOL', 'INT', 'FLOAT', 'STRING', 'TIMER'],
                      optional, auto-detect, default=None)
        """
        prop = self._blendobj.game.properties
        if ptype == None:
            # Detect the type (class name upper case)
            ptype = value.__class__.__name__.upper()
        if ptype == 'STR':
            # Blender property string are called 'STRING' (and not 'str' as in Python)
            ptype = 'STRING'
        prop[pid].type = ptype
        prop[pid].value = value
        return prop[pid]

    def _get_selected(self, name):
        """ get_selected returns the object with the name ``name`` from the
        selected objects list (usefull after appending)
        ie. importing a second object will be named "`name`.001" etc.

        :param name: name of the object
        :return: the first Blender object for which his name strats with the
        param `name` from those selected (imported object are selected)
        """
        for obj in bpy.context.selected_objects:
            if obj.name == name:
                return obj
        # fix Blender shorten the name
        # ie. 'torso_lift_armature' -> 'torso_lift_armatu.000'
        test_prefix = name[:17] + '.'
        # look for candidates
        candidates = [obj for obj in bpy.context.selected_objects if \
                      obj.name.startswith(test_prefix)]
        if len(candidates) > 0:
            if len(candidates) > 1:
                logger.warning(test_prefix + ": more than 1 candidate: " + candidate)
            return candidates[0]
        else:
            logger.warning(test_prefix + ": no candidate in " + \
                           str(bpy.context.selected_objects))
            return None

    def configure_mw(self, mw, config=None, method=None, path=None, component=None):
        """

        :param component: if set, force to use the configuration of the given
        component, instead of our own (default=None).
        """
        if not component:
            component = self._blendname
        # Configure the datastream for this component
        if not config:
            if not method:
                try:
                    config = MORSE_DATASTREAM_DICT[mw][component]
                    # TODO self._blendobj.game.properties["Class"].value ?
                    #      as map-key (in data, instead of blender-filename)
                except KeyError:
                    logger.warning("%s: default datastream method"%self.name)
                    # set the default method either it is an Actuator or a Sensor
                    method = 'XXX_message'
                    prop = self._blendobj.game.properties
                    if 'Path' in prop:
                        if prop['Path'].value.startswith('morse/sensors/'):
                            method = 'post_message'
                        elif prop['Path'].value.startswith('morse/actuators/'):
                            method = 'read_message'
                    config = [MORSE_DATASTREAM_MODULE[mw], method]
            else:
                if not path:
                    try:
                        path = MORSE_DATASTREAM_DICT[mw][component][2]
                        config = [MORSE_DATASTREAM_MODULE[mw], method, path]
                    except KeyError:
                        config = [MORSE_DATASTREAM_MODULE[mw], method]
                else:
                    config = [MORSE_DATASTREAM_MODULE[mw], method, path]
        Configuration().link_datastream(self, config)

    def configure_service(self, mw, component = None, config=None):
        if not component:
            component = self
        if not config:
            config = MORSE_SERVICE_DICT[mw]
        Configuration().link_service(component, config)

    def configure_modifier(self, mod, config=None):
        # Configure the modifier for this component
        if not config:
            config = MORSE_MODIFIER_DICT[mod][self._blendname]
        Configuration().link_modifier(self, config)

    def configure_overlay(self, mw, overlay, config=None):
        if not config:
            config = MORSE_SERVICE_DICT[mw]
        Configuration().link_overlay(self, config, overlay)

class timer(float):
    __doc__ = "this class extends float for the game properties configuration"
