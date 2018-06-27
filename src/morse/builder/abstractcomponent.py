import logging; logger = logging.getLogger("morsebuilder." + __name__)
import os
import pprint
import copy

from morse.builder import bpymorse
from morse.builder.data import *
from morse.core.exceptions import MorseBuilderUnexportableError, MorseBuilderError

from morse.helpers.loading import get_class, load_module_attribute

class Configuration(object):
    datastream = {}
    stream_manager = {}
    modifier = {}
    service = {}
    overlay = {}
    frequency = {}

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
        Configuration._update_name(old_name, new_name, Configuration.frequency)
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

    def link_stream_manager_config(manager, kwargs):
        Configuration.stream_manager.setdefault(manager, {}).update(kwargs)

    def has_datastream_configuration(component, stream):
        try:
            confs = Configuration.datastream[component.name]
            for conf in confs:
                if conf[0] == MORSE_DATASTREAM_MODULE[stream]:
                    return True
        except KeyError:
            return False

    def has_service_configuration(component, stream):
        try:
            confs = Configuration.service[component.name]
            for conf in confs:
                if conf[0] == MORSE_SERVICE_DICT[stream]:
                    return True
        except KeyError:
            return False

    def set_frequency(component, freq):
        Configuration.frequency[component.name] = freq

    def max_frequency():
        """ Returns the highest update frequency requested in the
        Builder script for any component (via component.frequency(...)).
        If no specific frequency has been set, returns MORSE's default
        (60Hz) """
        values = Configuration.frequency.values()
        if not values:
            return 60
        else:
            return max(values)

    def _remove_entries(dict_, robot_list):
        if robot_list is None:
            return dict_
        else:
            res = {}
            for k, v in dict_.items():
                for robot in robot_list:
                    if k.startswith(robot):
                        res[k] = v
                        break
            return res


    def write_config(robot_list):
        """ Write the 'component_config.py' file with the supplied settings """
        if not 'component_config.py' in bpymorse.get_texts().keys():
            last_text = bpymorse.new_text()
            last_text.name = 'component_config.py'
        cfg = bpymorse.get_text('component_config.py')
        cfg.clear()
        cfg.write('component_datastream = ' + pprint.pformat(
            Configuration._remove_entries(Configuration.datastream, robot_list)))
        cfg.write('\n')
        cfg.write('component_modifier = ' + pprint.pformat(
            Configuration._remove_entries(Configuration.modifier, robot_list)))
        cfg.write('\n')
        cfg.write('component_service = ' + pprint.pformat(
            Configuration._remove_entries(Configuration.service, robot_list)))
        cfg.write('\n')
        cleaned_overlays = {}
        for k, v in Configuration.overlay.items():
            cleaned_overlays[k] = Configuration._remove_entries(v, robot_list)
        cfg.write('overlays = ' + pprint.pformat(cleaned_overlays))
        cfg.write('\n')
        cfg.write('stream_manager = ' + pprint.pformat(Configuration.stream_manager))
        cfg.write('\n')

class AbstractComponent(object):

    components = [] # set of all created components

    def __init__(self, obj=None, filename='', category=''):
        self.set_blender_object(obj)
        self._resource_filename = filename # filename for datastream configuration
        self.has_urdf = True if self._resource_filename.endswith(".urdf") else False

        self._category = category # for morseable
        self.basename = None
        self.children = []
        self._exportable = True

        AbstractComponent.components.append(self)

    def set_blender_object(self, obj):
        if obj: # Force matrix_parent_inverse to identity #139
            obj.matrix_parent_inverse.identity()
            # make sure the object is visible in the viewport
            # otherwise it can prevent from updating its properties
            obj.hide = False
        self._bpy_object = obj # bpy object

    def append(self, obj, child = None, level=1):
        """ Add a child to the current object

        Add the object given as an argument as a child of this object. The
        argument is an instance to another component. This method is generally
        used to add components to a robot.
        *e.g.*, : robot.append(sensor), will set the robot parent of the sensor.

        If child is not None, the object will be parented to the named
        child of self instead of the root of self.
        """
        obj._bpy_object.matrix_parent_inverse.identity()
        if child:
            _child = self.get_child(child)
            if _child:
                obj._bpy_object.parent = _child
            else:
                raise MorseBuilderError("No such child %s" % child)
        else:
            obj._bpy_object.parent = self._bpy_object
        obj.parent = self
        self.children.append(obj)

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


    @staticmethod
    def close_context(level = 1):
        import inspect
        try:
            frame = inspect.currentframe()
            builderscript_frame = inspect.getouterframes(frame)[level][0] # parent frame
            cmpts = builderscript_frame.f_locals

            for name, component in cmpts.items():
                if isinstance(component, AbstractComponent):

                    if hasattr(component, "parent"):
                        continue

                    # do automatic renaming only if a name is not already manually set
                    # component.name accessor set both basename and bpy.name,
                    # which is the correct behaviour here. The bpy_name may be
                    # rewritten by _rename_tree, to get the correct hierarchy.
                    if not component.basename:
                        Configuration.update_name(component.name, name)
                        component.name = name

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
        """ Translate the current object

        The translation will add (x, y, z) to the current object location.
        default: x=0, y=0, z=0, unit: meter
        """
        old = self._bpy_object.location
        self._bpy_object.location = (old[0]+x, old[1]+y, old[2]+z)
    def rotate(self, x=0.0, y=0.0, z=0.0):
        """ Rotate the current object

        The rotation is an euler rotation relative to the object's center.
        default: x=0, y=0, z=0, unit: radian
        """
        old = self._bpy_object.rotation_euler
        self._bpy_object.rotation_euler = (old[0]+x, old[1]+y, old[2]+z)

    def property_value(self, name):
        try:
            return self._bpy_object.game.properties[name].value
        except KeyError:
            return None

    def properties(self, **kwargs):
        """ Add/modify the game properties of the Blender object

        Usage example:

        .. code-block:: python

            self.properties(capturing = True, classpath='module.Class', speed = 5.0)

        will create and/or set the 3 game properties Component_Tag, classpath, and
        speed at the value True (boolean), 'module.Class' (string), 5.0 (float).
        In Python the type of numeric value is 'int', if you want to force it to
        float, use the following: float(5) or 5.0
        Same if you want to force to integer, use: int(a/b)
        For the TIMER type, see the class timer(float) defined in this module:

        .. code-block:: python

            self.properties(my_clock = timer(5.0), my_speed = int(5/2))

        """
        bpymorse.properties(self._bpy_object, **kwargs)

    def select(self):
        bpymorse.select_only(self._bpy_object)

    def get_child(self, name, objects=None, recursive=True):
        """ get_child returns the child named :param name:

        If several children match the name, a warning is printed and
        the first one is returned.

        :param name: the textual name of the child
        :param objects: if specified, look for the child in this list of bpy Objects
        :param recursive: (default: True) if true, search for the child recursively
        """
        if objects is None:
            objects = self._bpy_object.children
        for obj in objects:
            if obj.name == name:
                return obj
        # fix Blender shorten the name
        # ie. 'torso_lift_armature' -> 'torso_lift_armatu.000'
        test_prefix = name.rsplit('.')[0]
        # look for candidates
        candidates = [obj for obj in objects \
                      if obj.name.startswith(test_prefix)]
        if len(candidates) > 0:
            if len(candidates) > 1:
                logger.warning(test_prefix + ": more than 1 candidate: " + \
                               str(candidates))
            return candidates[0]

        # nothing found yet. Start to search recursively:
        if recursive:
            for obj in objects:
                found = self.get_child(name, obj.children, recursive)
                if found: return found

        return None

    def _compute_direction(self, classpath):
        klass = get_class(classpath)

        from morse.core.actuator import Actuator
        from morse.core.sensor import Sensor

        if klass:
            if issubclass(klass, Actuator):
                return 'IN'
            elif issubclass(klass, Sensor):
                return 'OUT'
            else:
                logger.error("%s: no direction is precised nor can be "
                        "computed automatically." % classpath)
                return None


    def add_stream(self, datastream, method=None, path=None, classpath=None, direction=None, **kwargs):
        """ Add a data stream interface to the component

        Do the binding between a component and the method to export/import its
        data. This must be used in general by sensors and actuators. A single
        component can make several calls to this function to add bindings with
        more than one middleware.

        :param datastream: enum in ['ros', 'socket', 'yarp', 'text', 'pocolibs', 'moos']
        :param classpath: if set, force to use the configuration of the given
                          component, instead of our own (default=None).

        You can pass other argument to this method, they will be added as a map
        to the configuration.

        .. code-block:: python

            component.add_stream('ros', topic='/myrobots/data')
            component.add_stream('moos', moos_host='127.0.0.1', moos_port=9000,
                                    moos_name='iMorse')

        """
        self._err_if_not_exportable()

        if not classpath:
            classpath = self.property_value("classpath")

        if not classpath:
            logger.error("%s: no classpath defined for this "
                         "component! Check component definition " % self.name)
            return

        level = self.property_value("abstraction_level") or "default"

        if not direction:
            direction = self._compute_direction(classpath)
            if not direction:
                return

        config = []
        # Configure the datastream for this component
        if not method:
            if not classpath in MORSE_DATASTREAM_DICT:
                klass = get_class(classpath)

                from morse.core.actuator import Actuator
                from morse.core.sensor import Sensor
                # Check if we can use default interface...
                if klass and \
                   issubclass(klass, Actuator) and \
                   datastream in INTERFACE_DEFAULT_IN:

                    logger.warning("%s: no interfaces available for this "
                                   "component! Trying to use default one "
                                   "for %s." % (classpath, datastream))
                    config = [INTERFACE_DEFAULT_IN[datastream]]

                elif klass and \
                     issubclass(klass, Sensor) and \
                     datastream in INTERFACE_DEFAULT_OUT:

                    logger.warning("%s: no interfaces available for this "
                                   "component! Trying to use default one "
                                   "for %s." % (classpath, datastream))
                    config = [INTERFACE_DEFAULT_OUT[datastream]]

                else:
                    logger.error("%s: no interfaces available for this component!"
                                " Check builder/data.py." % classpath)
                    return

            else:
                interfaces = MORSE_DATASTREAM_DICT[classpath]

                if not level in interfaces:

                    if level == "default": # we need to look for the default level
                        module_name, class_name = classpath.rsplit('.', 1)
                        klass = load_module_attribute(module_name, class_name)

                        if not hasattr(klass, "_levels"):
                            logger.error("Component <%s> does not declare any "
                                        "default interface. You must call "
                                        "`add_stream` with an explicit method "
                                        "and Python module." % str(classpath))
                            return

                        # iterate over levels to find the one with the default flag
                        for key, value in klass._levels.items():
                            if value[2]:
                                level = key
                                # set the right default level
                                self.properties(abstraction_level = level)
                                logger.info("Using default level <%s> for "
                                            "component <%s>" % (level, classpath))
                                break

                        if level == "default":
                            logger.error("Component <%s> does not declare any"
                                        "default interface, and none of its "
                                        "abstraction levels is marked as the "
                                        "default one. You must call `add_stream`"
                                        " with an explicit method and Python "
                                        "module." % str(classpath))
                            return

                        if not level in interfaces:
                            logger.error("%s: no interfaces defined for this "
                                        "component for abstraction level <%s>!"
                                        "Check builder/data.py." %
                                        (classpath, level))
                            return

                    else:
                        logger.error("%s: no interfaces defined for this component"
                                    "for abstraction level <%s>! Check "
                                    "builder/data.py." %
                                    (classpath, level))

                interfaces = interfaces[level]
                if not datastream in interfaces:
                    logger.error("%s: no %s interface defined for this component "
                                "for abstraction level <%s>! "
                                "Check builder/data.py." %
                                (classpath, datastream, level))
                    return

                config = interfaces[datastream]
                if isinstance(config, list):
                    config = config[0]

                if config == INTERFACE_DEFAULT_OUT:
                    config = INTERFACE_DEFAULT_OUT[datastream]
                if config == INTERFACE_DEFAULT_IN:
                    config = INTERFACE_DEFAULT_IN[datastream]
                if isinstance(config, str):
                    config = [config]

        elif not path:
            config = [method]
        else:
            config = [method, path]

        if datastream in MORSE_DATASTREAM_MODULE:
            datastream_classpath = MORSE_DATASTREAM_MODULE[datastream]
        else:
            datastream_classpath = datastream


        config.insert(0, datastream_classpath)
        config.append(direction)
        config.append(kwargs) # append additional configuration (eg. topic name)
        Configuration.link_datastream(self, config)

    def add_service(self, interface, component=None, config=None):
        """ Add a service interface to the component

        Similar to the previous function. Its argument is the name of the
        interface to be used.
        """

        if not component:
            component = self
        if not config:
            config = MORSE_SERVICE_DICT[interface]
        Configuration.link_service(component, config)

    def add_interface(self, interface, **kwargs):
        """ Add a service and stream interface to the component

        Its argument is the name of the interface to be used.
        """
        self.add_service(interface)
        if self._exportable:
            self.add_stream(interface, **kwargs)

    def alter(self, modifier_name=None, classpath=None, direction=None, **kwargs):
        """ Add a modifier specified by its first argument to the component """
        # Configure the modifier for this component
        config = []
        obj_classpath = self.property_value('classpath')
        if not direction:
            direction = self._compute_direction(obj_classpath)
            if not direction:
                return
        if not classpath:
            classpath = MORSE_MODIFIER_DICT[modifier_name][obj_classpath]
        config.append(classpath)
        config.append(direction)
        config.append(kwargs)
        Configuration.link_modifier(self, config)

    def add_overlay(self, datastream, overlay, config=None, **kwargs):
        """ Add a service overlay for a specific service manager to the component

        Similar to the add_stream function. Its argument is the name of the
        datastream to be used.
        """
        if not config:
            config = MORSE_SERVICE_DICT[datastream]
        Configuration.link_overlay(self, config, overlay, kwargs)

    def level(self, level):
        """ Set the 'realism level' of the component.

        Some components define several abstraction level that impact what data
        are exported.

        See each component documentation for the list of available levels.
        """
        self.properties(abstraction_level = level)

    def _set_sensor_frequency(self, sensor, delay):
        if bpymorse.version() >= (2, 74, 5):
            sensor.tick_skip = delay
        else:
            sensor.frequency = delay

    def frequency(self, frequency=None):
        """ Set the frequency of the Python module

        :param frequency: (int) Desired frequency,
            0 < frequency < logic tics
        """
        if frequency:
            Configuration.set_frequency(self, frequency)
            self.properties(frequency = frequency)

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
                               enum in ['calling.sensor_action',
                                        'calling.actuator_action',
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
        bpymorse.add_sensor(type='ALWAYS', name='MORSE_LOGIC')
        sensor = self._bpy_object.game.sensors[-1]
        sensor.use_pulse_true_level = True
        bpymorse.add_controller(type='PYTHON')
        controller = self._bpy_object.game.controllers[-1]
        controller.mode = 'MODULE'
        controller.module = calling_module
        controller.link(sensor = sensor)

    def _compute_filepath(self, component):
        if component.endswith('.blend'):
            filepath = os.path.abspath(component) # external blend file
        else:
            filepath = os.path.join(MORSE_COMPONENTS, self._category,
                                    component + '.blend')
        looked_dirs = [filepath]

        if not os.path.exists(filepath):
            # Search for some blend file in different paths
            filepath = None
            resource_path = MORSE_RESOURCE_PATH.split(os.pathsep)
            for path in resource_path:
                tmp = os.path.join(path, component)
                looked_dirs.append(tmp)
                if os.path.exists(tmp):
                    filepath = tmp
                    break
            # Check if we got a match

            if not filepath:
                logger.error("Error while trying to load '%s': model not found.\n"
                             "I was looking for one of these files: \n%s\n"
                             "Either provide an absolute path, or a path relative \n"
                             "to MORSE assets directories ($MORSE_RESOURCE_PATH \n"
                             "or default path, typically $PREFIX/share/morse/data)."% (component, looked_dirs))
                raise FileNotFoundError("%s '%s' not found"%(self.__class__.__name__, component))

        return filepath

    def load_urdf(self):
        """
        Loads the URDF file passed to the AbstractComponent constructor.

        :return: the root object created from the URDF file.
        """
        if not self.has_urdf:
            return []

        from morse.builder.urdf import URDF

        self.urdf = URDF(self._resource_filename)
        return self.urdf.build()

    def append_meshes(self, objects=None, component=None, prefix=None):
        """ Append the component's Blender objects to the scene

        The ``objects`` are located either in:
        MORSE_COMPONENTS/``self._category``/``component``.blend/Object/
        or in: MORSE_RESOURCE_PATH/``component``/Object/

        If `component` is not set (neither as argument of `append_meshes` nor
        through the :py:class:`AbstractComponent` constructor), a Blender
        `Empty` is created instead.

        .. note::

            By default, all the objects present in the component's blend file are
            imported. If you need to exclude some (like lights you may have in your
            blend file), prefix the name of this objects with ``_`` so that MORSE
            ignores them.

        :param objects: list of the objects names to append
        :param component: component in which the objects are located
        :param prefix: filter the objects names to append (used by PassiveObject)
        :return: list of the imported (selected) Blender objects
        """

        if component is None and self.has_urdf:
            logger.warning("calling AbstractComponent.append_meshes with a URDF component. Skipping it.")
            return []

        component = component or self._resource_filename

        if not component: # no Blender resource: simply create an empty
            bpymorse.deselect_all()
            bpymorse.add_morse_empty()
            return [bpymorse.get_first_selected_object(),]

        filepath = self._compute_filepath(component)

        if not objects: # append all objects from blend file
            objects = bpymorse.get_objects_in_blend(filepath)

        filtered_objects = objects

        # ignore objects starting with '_'
        filtered_objects = [obj for obj in objects if not obj.startswith('_')]

        if prefix: # filter (used by PassiveObject)
            filtered_objects = [obj for obj in filtered_objects if obj.startswith(prefix)]

        logger.info("Importing objects from %s: %s" % (filepath,
                                    str(objects)))
        excluded = set(objects) - set(filtered_objects)
        if excluded:
            logger.info("(excluding %s)" % str(list(excluded)))

        # Format the objects list to append
        objlist = [{'name':obj} for obj in filtered_objects]

        bpymorse.deselect_all()
        # Append the objects to the scene, and (auto)select them
        if bpymorse.version() >= (2, 71, 6):
            bpymorse.append(directory=filepath + '/Object/',
                            autoselect=True, files=objlist)
        else:
            bpymorse.link_append(directory=filepath + '/Object/', link=False,
                                 autoselect=True, files=objlist)

        return bpymorse.get_selected_objects()

    def append_scenes(self, component=None):
        component = component or self._resource_filename

        filepath = self._compute_filepath(component)

        scenes = bpymorse.get_scenes_in_blend(filepath)

        # Format the objects list to append
        sclist = [{'name':sc} for sc in scenes]

        bpymorse.deselect_all()
        # Append the objects to the scene, and (auto)select them
        if bpymorse.version() >= (2, 71, 6):
            bpymorse.append(directory=filepath + '/Scene/',
                            autoselect=True, files=sclist)
        else:
            bpymorse.link_append(directory=filepath + '/Scene/', link=False,
                                 autoselect=True, files=sclist)

    def append_collada(self, component=None):
        """ Append Collada objects to the scene

        :param component: component in which the objects are located
        :return: list of the imported Blender objects
        """
        if not component:
            component = self._resource_filename

        if component.endswith('.dae'):
            filepath = os.path.abspath(component) # external blend file
        else:
            filepath = os.path.join(MORSE_COMPONENTS, self._category,
                                    component + '.dae')

        if not os.path.exists(filepath):
            logger.error("Collada file %s for external asset import can" \
                         "not be found.\nEither provide an absolute path, or" \
                         "a path relative to MORSE assets directory (typically"\
                         "$PREFIX/share/morse/data)" % filepath)
            return

        # Save a list of objects names before importing Collada
        objects_names = [obj.name for obj in bpymorse.get_objects()]
        # Import Collada from filepath
        bpymorse.collada_import(filepath=filepath)
        # Get a list of the imported objects
        imported_objects = [obj for obj in bpymorse.get_objects() \
                            if obj.name not in objects_names]

        return imported_objects

    def _make_transparent(self, obj, alpha):
        obj.game.physics_type = 'NO_COLLISION'
        for m in obj.material_slots:
            m.material.use_transparency = True
            m.material.alpha = alpha
            m.material.transparency_method = 'Z_TRANSPARENCY'
        for c in obj.children:
            self._make_transparent(c, alpha)

    def _set_color(self, obj, rgb):
        if len(obj.material_slots) > 0:
            obj.active_material.diffuse_color = rgb
        for c in obj.children:
            self._set_color(c, rgb)

    def set_color(self, r, g, b):
        """ Set the color of the component.

        Will try to apply the RGB color to the active material
        of all the meshes that are children of this component.
        """
        if self._bpy_object:
            self._set_color(self._bpy_object, (r,g,b))

    def profile(self):
        """ Watch the average time used during the simulation.

        Display the profile of the component on the viewport in percent.
        As Blender would for framerate and other debug-properties.
        """
        if self._category is not 'sensors':
            logger.warning("profile currently supports only sensors (%s)"%self)
        for key in ["profile", "profile_action", "profile_modifiers",
                    "profile_datastreams"]:
            prop = bpymorse._property_new(self._bpy_object, key, "0")
            prop.show_debug = True
        bpymorse.get_context_scene().game_settings.show_debug_properties = True

    def __str__(self):
        return self.name

    def is_exportable(self):
        return self._exportable

    def mark_unexportable(self):
        self._exportable = False

    def _err_if_not_exportable(self):
        if not self._exportable:
            raise MorseBuilderUnexportableError(self.name)

class timer(float):
    __doc__ = "this class extends float for the game properties configuration"
