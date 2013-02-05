import logging; logger = logging.getLogger("morsebuilder." + __name__)
import os
import json
from morse.builder import bpymorse
import copy

from morse.builder.data import *

class Configuration(object):
    datastream = {}
    modifier = {}
    service = {}
    overlay = {}

    def link_datastream(component, datastream_cfg):
        Configuration.datastream.setdefault(component.name, []).append(datastream_cfg)

    def _update_name(old_name, new_name, dict):
        """ Updated components name after renaming.

        TODO: since we update the dict 'inplace' we will run into
        trouble if an 'old' name matches a 'new' name...
        """
        try:
            dict[new_name] = dict.pop(old_name)
        except KeyError:
            pass

    def update_name(old_name, new_name):
        Configuration._update_name(old_name, new_name, Configuration.datastream)
        Configuration._update_name(old_name, new_name, Configuration.modifier)
        Configuration._update_name(old_name, new_name, Configuration.service)
        for k,v in Configuration.overlay.items():
            Configuration._update_name(old_name, new_name, v)

    def link_service(component, service_cfg):
        # Special case here for the pseudo component 'simulation' that
        # covers the simulator management services.
        if component == "simulation":
            Configuration.service.setdefault(component, []).append(service_cfg)
        else:
            Configuration.service.setdefault(component.name, []).append(service_cfg)

    def link_modifier(component, modifier_cfg):
        Configuration.modifier.setdefault(component.name, []).append(modifier_cfg)

    def link_overlay(component,  manager, overlay_cfg, kwargs):
        Configuration.overlay.setdefault(manager, {})[component.name] = [overlay_cfg, kwargs]

    def write_config():
        """ Write the 'component_config.py' file with the supplied settings """
        if not 'component_config.py' in bpymorse.get_texts().keys():
            bpymorse.new_text()
            bpymorse.get_last_text().name = 'component_config.py'
        cfg = bpymorse.get_text('component_config.py')
        cfg.clear()
        cfg.write('component_datastream = ' + json.dumps(Configuration.datastream, indent=1) )
        cfg.write('\n')
        cfg.write('component_modifier = ' + json.dumps(Configuration.modifier, indent=1) )
        cfg.write('\n')
        cfg.write('component_service = ' + json.dumps(Configuration.service, indent=1) )
        cfg.write('\n')
        cfg.write('overlays = ' + json.dumps(Configuration.overlay, indent=1) )
        cfg.write('\n')

class AbstractComponent(object):

    components = [] # set of all created components

    def __init__(self, obj=None, filename='', category=''):
        self.set_blender_object(obj)
        self._blender_filename = filename # filename for datastream configuration
        self._category = category # for morseable
        self.basename = None
        self.children = []

        AbstractComponent.components.append(self)

    def set_blender_object(self, obj):
        if obj: # Force matrix_parent_inverse to identity #139
            obj.matrix_parent_inverse.identity()
            # make sure the object is visible in the viewport
            # otherwise it can prevent from updating its properties
            obj.hide = False
        self._bpy_object = obj # bpy object

    def append(self, obj, level=1):
        """ Add a child to the current object

        eg: robot.append(sensor), will set the robot parent of the sensor.
        """
        obj._bpy_object.matrix_parent_inverse.identity()
        obj._bpy_object.parent = self._bpy_object
        obj.parent = self
        self.children.append(obj)

        #TODO: replace by sys._getframes() ??
        import inspect
        try:
            frame = inspect.currentframe()
            builderscript_frame = inspect.getouterframes(frame)[level][0] # parent frame
            cmpts = builderscript_frame.f_locals
            if "self" in  cmpts: #some silly guy decided to write a class to describe a silly robot
                tmp = copy.copy(cmpts["self"].__dict__)
                tmp.update(cmpts)
                cmpts = tmp

            for name, component in cmpts.items():
                if component == obj:
                    if not component.basename: # do automatic renaming only if a name is not already manually set
                        component.basename = name
        finally:
            del builderscript_frame
            del frame

    @property
    def name(self):
        return self._bpy_object.name
    @name.setter
    def name(self, value):
        if value:
            if '.' in value:
                raise SyntaxError("Invalid component name: <%s>. Dots are not allowed." % value)
            self.basename = value
            self._bpy_object.name = value
    @property
    def location(self):
        return self._bpy_object.location
    @location.setter
    def location(self, xyz):
        self._bpy_object.location = xyz
    @property
    def scale(self):
        return self._bpy_object.scale
    @scale.setter
    def scale(self, xyz):
        self._bpy_object.scale = xyz
    @property
    def rotation_euler(self):
        return self._bpy_object.rotation_euler
    @rotation_euler.setter
    def rotation_euler(self, xyz):
        self._bpy_object.rotation_euler = xyz
    def translate(self, x=0.0, y=0.0, z=0.0):
        """ Location of the object

        float array of 3 items in [-inf, inf], default (0.0, 0.0, 0.0)
        """
        old = self._bpy_object.location
        self._bpy_object.location = (old[0]+x, old[1]+y, old[2]+z)
    def rotate(self, x=0.0, y=0.0, z=0.0):
        """ Rotation in Eulers

        float array of 3 items in [-inf, inf], default (0.0, 0.0, 0.0)
        """
        old = self._bpy_object.rotation_euler
        self._bpy_object.rotation_euler = (old[0]+x, old[1]+y, old[2]+z)
    def property_value(name):
        return self._bpy_object.game.properties[name].value
    def properties(self, **kwargs):
        """ Add/modify the game properties of the Blender object

        Usage example:

        .. code-block:: python

            self.properties(Component_Tag = True, classpath='module.Class', speed = 5.0)

        will create and/or set the 3 game properties Component_Tag, classpath, and
        speed at the value True (boolean), 'module.Class' (string), 5.0 (float).
        In Python the type of numeric value is 'int', if you want to force it to
        float, use the following: float(5) or 5.0
        Same if you want to force to integer, use: int(a/b)
        For the TIMER type, see the class timer(float) defined in this module:

        .. code-block:: python

            self.properties(My_Clock = timer(5.0), My_Speed = int(5/2))

        """
        prop = self._bpy_object.game.properties
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
        self.select()
        bpymorse.new_game_property()
        prop = self._bpy_object.game.properties
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
        prop = self._bpy_object.game.properties
        if ptype == None:
            # Detect the type (class name upper case)
            ptype = value.__class__.__name__.upper()
        if ptype == 'STR':
            # Blender property string are called 'STRING' (and not 'str' as in Python)
            ptype = 'STRING'
        prop[pid].type = ptype
        prop[pid].value = value
        return prop[pid]

    def select(self):
        bpymorse.select_only(self._bpy_object)

    def get_child(self, name, objects=None):
        """ get_child returns the child named :param name: """
        if not objects:
            objects = self._bpy_object.children
        for obj in objects:
            if obj.name == name:
                return obj
        # fix Blender shorten the name
        # ie. 'torso_lift_armature' -> 'torso_lift_armatu.000'
        test_prefix = name[:17] + '.'
        # look for candidates
        candidates = [obj for obj in objects \
                      if obj.name.startswith(test_prefix)]
        if len(candidates) > 0:
            if len(candidates) > 1:
                logger.warning(test_prefix + ": more than 1 candidate: " + \
                               str(candidates))
            return candidates[0]
        else:
            logger.warning(test_prefix + ": no candidate in " + str(objects))

        return None

    def configure_mw(self, datastream, method=None, path=None, component=None):
        logger.warning("configure_mw is deprecated, use add_stream instead")
        return self.add_stream(datastream, method, path, component)

    def add_stream(self, datastream, method=None, path=None, component=None, **kwargs):
        """ add a data stream interface to the component

        :param component: if set, force to use the configuration of the given
        component, instead of our own (default=None).

        You can pass other argument to this method, they will be added as a map
        to the configuration.
        component.add_stream('ros', topic='/myrobots/data')
        """
        if not component:
            component = self._blender_filename
        config = []
        # Configure the datastream for this component
        if not method:
            try:
                config = MORSE_DATASTREAM_DICT[datastream][component][:]
            except KeyError:
                logger.error("%s: no default datastream method [%s][%s]" % \
                             (self.name, datastream, component))
                return
        elif not path:
            config = [method]
        else:
            config = [method, path]

        config.insert(0, MORSE_DATASTREAM_MODULE[datastream])
        config.append(kwargs) # append additional configuration (eg. topic name)
        Configuration.link_datastream(self, config)

    def configure_service(self, interface, component=None, config=None):
        logger.warning("configure_service is deprecated, use add_service instead")
        return self.add_service(interface, component, config)

    def add_service(self, interface, component=None, config=None):
        if not component:
            component = self
        if not config:
            config = MORSE_SERVICE_DICT[interface]
        Configuration.link_service(component, config)

    def add_interface(self, interface, **kwargs):
        self.add_service(interface)
        self.add_stream(interface, **kwargs)

    def configure_modifier(self, mod, config=None):
        # Configure the modifier for this component
        if not config:
            config = MORSE_MODIFIER_DICT[mod][self._blender_filename]
        Configuration.link_modifier(self, config)

    def configure_overlay(self, datastream, overlay, config=None, **kwargs):
        if not config:
            config = MORSE_SERVICE_DICT[datastream]
        Configuration.link_overlay(self, config, overlay, kwargs)

    def frequency(self, frequency=None, delay=0):
        """ Set the frequency delay for the call of the Python module

        :param frequency: (int) Desired frequency,
            0 < frequency < logic tics
        :param delay: (int) Delay between repeated pulses
            (in logic tics, 0 = no delay)
            if frequency is set, delay is obtained by fps / frequency.
        """
        if frequency:
            delay = max(0, bpymorse.get_fps() // frequency - 1)
        sensors = [s for s in self._bpy_object.game.sensors if s.type == 'ALWAYS']
        if len(sensors) > 1:
            logger.warning(self.name + " has too many Game Logic sensors to "+\
                    "tune its frequency, change it through Blender")
        elif len(sensors) > 0:
            sensors[0].frequency = delay
        else:
            logger.warning(self.name + " has no 'ALWAYS' Game Logic sensor. "+\
                           "Unable to tune its frequency.")

    def is_morseable(self):
        for sensor in self._bpy_object.game.sensors:
            if sensor.type == 'ALWAYS':
                for controller in sensor.controllers:
                    if controller.type == 'PYTHON' and \
                       controller.module.startswith("calling."):
                        return True

        return False

    def morseable(self, calling_module=None):
        """ Make this component simulable in MORSE

        :param calling_module: Module called each simulation cycle.
            enum in ['calling.sensor_action', 'calling.actuator_action',
                    'calling.robot_action']
        """
        if not calling_module:
            calling_module = 'calling.'+self._category[:-1]+'_action'
        # add default class to this component
        if calling_module == 'calling.robot_action':
            self.properties(Robot_Tag = True, classpath = 'morse.core.robot.Robot')
        elif calling_module == 'calling.sensor_action':
            self.properties(Component_Tag = True, classpath = 'morse.core.sensor.Sensor')
        elif calling_module == 'calling.actuator_action':
            self.properties(Component_Tag = True, classpath = 'morse.core.actuator.Actuator')
        else:
            logger.warning(self.name + ": unknown category: " + calling_module)

        # add Game Logic sensor and controller to simulate the component
        # Sensor: Always --- Controller: Python: module = 'calling.XXX_action'
        self.select()
        bpymorse.add_sensor(type='ALWAYS')
        sensor = self._bpy_object.game.sensors[-1]
        sensor.use_pulse_true_level = True
        bpymorse.add_controller(type='PYTHON')
        controller = self._bpy_object.game.controllers[-1]
        controller.mode = 'MODULE'
        controller.module = calling_module
        controller.link(sensor = sensor)

    def append_meshes(self, objects=None, component=None, prefix=None):
        """ Append the objects to the scene

        The `objects` are located in:
        MORSE_COMPONENTS/`self._category`/`component`.blend/Object/

        :param objects: list of the objects names to append
        :param component: component in which the objects are located
        :return: list of the imported (selected) Blender objects
        """
        if not component:
            component = self._blender_filename

        if component.endswith('.blend'):
            filepath = os.path.abspath(component) # external blend file
        else:
            filepath = os.path.join(MORSE_COMPONENTS, self._category, \
                                    component + '.blend')

        if not os.path.exists(filepath):
            # Search for some blend file in different paths
            filepath = None
            resource_path = MORSE_RESOURCE_PATH.split(':')
            for path in resource_path:
                tmp = os.path.join(path, component)
                if os.path.exists(tmp):
                    filepath = tmp
                    break
            # Check if we got a match
            if not filepath:
                logger.error("Blender file '%s' for external asset import can" \
                             "not be found.\nEither provide an absolute path," \
                             " or a path relative to MORSE assets directory\n" \
                             "(typically $PREFIX/share/morse/data)"%component)
                return

        if not objects: # link_append all objects from blend file
            objects = bpymorse.get_objects_in_blend(filepath)

        if prefix: # filter (used by PassiveObject)
            objects = [obj for obj in objects if obj.startswith(prefix)]

        # Format the objects list for link_append
        objlist = [{'name':obj} for obj in objects]

        bpymorse.deselect_all()
        # Append the objects to the scene, and (auto)select them
        bpymorse.link_append(directory=filepath + '/Object/', link=False,
                             autoselect=True, files=objlist)

        return bpymorse.get_selected_objects()

    def append_collada(self, component=None):
        """ Append Collada objects to the scene

        The objects are located in:
        MORSE_COMPONENTS/`self._category`/`component`.dae

        :param component: component in which the objects are located
        :return: list of the imported Blender objects
        """
        if not component:
            component = self._blender_filename

        if component.endswith('.dae'):
            filepath = os.path.abspath(component) # external blend file
        else:
            filepath = os.path.join(MORSE_COMPONENTS, self._category, \
                                    component + '.dae')

        if not os.path.exists(filepath):
            logger.error("Collada file %s for external asset import can" \
                         "not be found.\nEither provide an absolute path, or" \
                         "a path relative to MORSE assets directory (typically"\
                         "$PREFIX/share/morse/data)" % (filepath))
            return

        # Save a list of objects names before importing Collada
        objects_names = [obj.name for obj in bpymorse.get_objects()]
        # Import Collada from filepath
        bpymorse.collada_import(filepath=filepath)
        # Get a list of the imported objects
        imported_objects = [obj for obj in bpymorse.get_objects() \
                            if obj.name not in objects_names]

        return imported_objects

    def profile(self):
        """ Watch the average time used during the simulation, in percent. """
        if self._category is not 'sensors':
            logger.warning("profile currently supports only sensors")
        prop = self._property_new("profile", "0 %")
        prop.show_debug = True
        prop = self._property_new("profile::action", "0 %")
        prop.show_debug = True
        prop = self._property_new("profile::modifiers", "0 %")
        prop.show_debug = True
        prop = self._property_new("profile::datastreams", "0 %")
        prop.show_debug = True
        bpymorse.get_context_scene().game_settings.show_debug_properties = True

    def __str__(self):
        return self.name

class timer(float):
    __doc__ = "this class extends float for the game properties configuration"
