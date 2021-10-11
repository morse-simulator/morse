import logging; logger = logging.getLogger("morsebuilder." + __name__)
import os
import pprint
from morse.core import mathutils
from morse.builder.morsebuilder import *
from morse.builder.data import MORSE_DATASTREAM_MODULE
from morse.builder.abstractcomponent import Configuration
from morse.core.morse_time import TimeStrategies
from morse.core import blenderapi
import json


class Environment(AbstractComponent):
    """ Class to configure the general environment of the simulation

    It handles the background environment in which your robots are simulated,
    general properties of the simulation, the default location and orientation
    of the camera, the Blender Game Engine settings, configure the parameters
    for the multi-node simulation and also writes the 'component_config.py' file.
    """
    multinode_distribution = dict()

    def __init__(self, filename, main_scene = None, fastmode = False, component_renaming = True,
                                                                      auto_tune_time = True):
        """
        :param fastmode: (default: False) if True, disable most visual
                         effects (like lights...) to get the fastest
                         running simulation.  Useful for unit-tests for
                         instance, or in simulations where realistic
                         environment texturing is not required (*e.g.*,
                         no video camera)
        :param component_renaming: (default: True): if True,
            automatically rename Blender object on the base of builder
            python object. It is the recommanded settings, and is mandatory
            if you use pymorse. Only set to False if you notice trouble with
            it, and report the issue to the Morse project
        :param auto_tune_time: (default: True): If True, Morse will try
            to compute a good setting for your simulation, on the basis on the
            described scene. The feature is automatically disabled if you make
            an explicit call to some time-related method, such as simulator_frequency.
            Note that it is an experimental feature, so disable it if you see
            problem with your simulation, and report it to the Morse project.
        """
        AbstractComponent.__init__(self, category = 'environments', filename = filename)
        if main_scene:
            base_scene = bpymorse.get_context_scene().name
            self.append_scenes()
            bpymorse.deselect_all()
            scene = bpymorse.set_active_scene(main_scene)
            for obj in scene.objects:
                obj.select = True
            bpymorse.make_links_scene(scene=base_scene)
            bpymorse.del_scene()
            bpymorse.set_active_scene(base_scene)
        else:
            self.append_meshes()
        AbstractComponent.components.remove(self) # remove myself from the list of components to ensure my destructor is called

        self._handle_default_interface()

        # Rename the components according to their variable names
        if component_renaming:
            self._rename_components()

        self.fastmode = fastmode
        self.auto_tune_time = auto_tune_time

        self._created = False
        self._camera_location = [5, -5, 5]
        self._camera_rotation = [0.7854, 0, 0.7854]
        self._focal_length = 20.0
        self._environment_file = filename
        self._multinode_configured = False
        self._display_camera = None
        self.is_material_mode_custom = False
        self._node_name = None
        self._physics_step_sub = 2
        # Add empty object holdings MORSE Environment's properties
        # for UTM modifier configutation ( uses env.properties(...) )
        bpymorse.deselect_all()
        bpymorse.add_morse_empty()
        obj = bpymorse.get_context_object()
        obj.name = 'MORSE.Properties'
        self.set_blender_object(obj)
        # Init. camera's properties
        self.set_camera_speed()
        self.set_camera_clip()

        self.set_gravity()

    def is_internal_camera(self, camera):
        return not self._multinode_configured or \
            self._node_name in self.multinode_distribution and \
            camera._bpy_object.parent.name in self.multinode_distribution[self._node_name]

    def _write_multinode(self, node_name):
        """ Configure this node according to its name
            and the multinode_distribution dictionnary.
        """
        if not self._multinode_configured:
            return
        if not node_name in self.multinode_distribution.keys():
            logger.warning("Node " + node_name + " is not defined in the " + \
                "env.multinode_distribution dict. It will manage no robot!")
            self.multinode_distribution[node_name] = []
        for obj in bpymorse.get_objects():
            p = obj.game.properties
            # Here we check that all objects declared for this node are robots
            if obj.name in self.multinode_distribution[node_name]:
                if not 'Robot_Tag' in p:
                    logger.warning(obj.name + " is not a robot!." + \
                        " Will not be published by this MORSE node.")
                else:
                    logger.info("Node " + node_name + \
                        " will publish robot" + obj.name)
            # Here, we make external other robots
            if 'Robot_Tag' in p:
                if not obj.name in self.multinode_distribution[node_name]:
                    logger.debug("Node " + node_name + \
                        " will not publish robot " + obj.name)
                    p['Robot_Tag'].name = 'External_Robot_Tag'
                    # Make external robots static
                    # This should stop them from drifting downwards
                    obj.game.physics_type = 'STATIC'

        """ Write the 'multinode_config.py' script """
        node_config = { 'protocol': self._protocol,
                        'node_name': node_name,
                        'server_address': self._server_address,
                        'server_port': self._server_port,}
        # Create the config file if it does not exist
        if not 'multinode_config.py' in bpymorse.get_texts().keys():
            bpymorse.new_text()
            bpymorse.get_last_text().name = 'multinode_config.py'
        cfg = bpymorse.get_text('multinode_config.py')
        cfg.clear()
        cfg.write('node_config = ' + pprint.pformat(node_config) )
        cfg.write('\n')

    def _rename_components(self):
        """ Rename Blender objects after the variable name used
        in the Builder script and there hierarchy.

        If a name is already set (with 'obj.name=...'), it is used as it,
        and only the hierarchy is added to the name.
        """
        import inspect
        frame = inspect.currentframe()
        frames = inspect.getouterframes(frame)
        size_stack = len(frames)
        for i in range(size_stack - 1, 0, -1):
            if frames[i][3] == '__init__':
                break
        del frame
        del frames

        AbstractComponent.close_context(i + 2)

        for component in AbstractComponent.components:
            if isinstance(component, Robot):

                def renametree(cmpt, fqn):
                    if not cmpt.basename:
                        return
#                        raise SyntaxError("You need to assign the component of type %s to a variable" %
#                                        cmpt)
                    list_name = [cmpt._bpy_object.name for cmpt in AbstractComponent.components if cmpt != component ]
                    fqn.append(cmpt.basename)
                    new_name = '.'.join(fqn)

                    i = 1
                    while new_name in list_name:
                        fqn.pop()
                        fqn.append("%s_%03d" % (cmpt.basename, i))
                        new_name = '.'.join(fqn)
                        i += 1

                    Configuration.update_name(cmpt.name, new_name)
                    cmpt._bpy_object.name = new_name
                    for child in cmpt.children:
                        renametree(child, fqn[:])

                renametree(component, [])

    def _configure_default_interface(self, component, interface):
        for child in component.children:
            if child.is_morseable():
                if not Configuration.has_datastream_configuration(child, interface) and \
                   child.is_exportable():
                    child.add_stream(interface)

                if not Configuration.has_service_configuration(child, interface):
                    child.add_service(interface)

                self._configure_default_interface(child, interface)

    def _handle_default_interface(self):
        """
        Handle the semantic of default interface.

        For each robot, for each of this children, if the child has no
        specific configuration for its 'default_interface', the function
        adds it automatically.
        """
        for component in AbstractComponent.components:
            if isinstance(component, Robot) and component.default_interface:
                self._configure_default_interface(component, component.default_interface)



    def place_camera(self, location):
        logger.warning("`place_camera` is deprecated, use `set_camera_location` instead")
        self.set_camera_location(location)

    def aim_camera(self, rotation):
        logger.warning("`aim_camera` is deprecated, use `set_camera_rocation` instead")
        self.set_camera_rotation(rotation)

    def set_camera_location(self, location):
        """ Set the location of the default camera.

        :param location: list with the new 3D coordinates for the camera.

        .. code-block:: python

            env.set_camera_location([10.0, -10.0, 3.0])

        """
        self._camera_location = location

    def set_camera_rotation(self, rotation):
        """ Set the orientation of the default camera

        :param rotation: list with an euler rotation for the camera.

        .. code-block:: python

            env.set_camera_rotation([1.3300, 0, 0.7854])

        """
        self._camera_rotation = rotation

    def set_camera_clip(self, clip_start=0.1, clip_end=100):
        """ Set the simulator's Camera near and far clipping distance

        :param clip_start: Camera near clipping distance, float in meters (default 0.1)
        :param clip_end: Camera far clipping distance, float in meters (default 100)
        """
        self._camera_clip_start = clip_start
        self._camera_clip_end   = clip_end

    def set_camera_speed(self, speed=2.0):
        """ Set the simulator's Camera speed

        :param speed: desired speed of the camera, in meter by second.
        """
        self._camera_speed = speed

    def set_camera_focal_length(self, focal_length=20.0):
        """ Set the focal length of the default camera

        :param focal_length: focal length im mm (default 20.0)

        .. code-block:: python

            env.set_camera_focal_length(50.0)

        """
        self._focal_length = focal_length

    def _cfg_camera_scene(self):
        scene = bpymorse.get_context_scene()
        scene.name = 'S.MORSE_LOGIC'
        from morse.builder.sensors import VideoCamera
        cfg_camera_scene = []
        for component in AbstractComponent.components:
            # do not create scene for external camera
            if isinstance(component, VideoCamera) and \
                    self.is_internal_camera(component):
                res_x = component.property_value('cam_width')
                res_y = component.property_value('cam_height')
                name = 'S.%dx%d' % (res_x, res_y)
                if not name in cfg_camera_scene:
                    # Create a new scene for the Camera
                    bpymorse.new_scene(type='LINK_OBJECTS')
                    scene = bpymorse.get_context_scene()
                    scene.name = name
                    scene.render.resolution_x = res_x
                    scene.render.resolution_y = res_y
                    scene.game_settings.physics_engine = 'NONE'
                    # TODO disable logic and physic in this created scene
                    cfg_camera_scene.append(name)

    def create(self, name=None):
        """ Generate the scene configuration and insert necessary objects

        Should always be called at the very end of the Builder script. It will
        finalise the building process and write the configuration files.
        """
        # Invoke special methods of component that must take place *after* renaming
        for component in AbstractComponent.components:
            if hasattr(component, "after_renaming"):
                component.after_renaming()

        # Compute node name
        if name is None:
            try:
                self._node_name = os.environ["MORSE_NODE"]
            except KeyError:
                import socket
                self._node_name = socket.gethostname()
        else:
            self._node_name = name

        # Check time properties
        scene = bpymorse.get_context_scene()
        base_frequency = scene.game_settings.fps
        max_frequency_requested = Configuration.max_frequency()
        time_scale = self.property_value('time_scale')
        if self.auto_tune_time:
            self.use_vsync('OFF')
            # notyet :D
            # self.use_internal_syncer()
            if max_frequency_requested > base_frequency:
                self.simulator_frequency(max_frequency_requested)
        else:
            # Just report bad looking configuration
            if max_frequency_requested > base_frequency:
                logger.warning("You are requesting a component at %d Hz, but the "
                               "simulator main loop is running only at %d Hz. Try "
                               "to raise the frequency of the simulation using "
                               "env.simulator_frequency(%d)" %
                               (max_frequency_requested, base_frequency, max_frequency_requested))
            if time_scale:
                real_fps_requested = max_frequency_requested * time_scale
                if real_fps_requested > 1000.0:
                    logger.warning("You are requesting a component at %d Hz, with "
                                   " time acceleration factor of %f, leading to a "
                                   " real frequency of %d Hz. It will probably hard "
                                   " to reach this value with Morse, so consider to "
                                   " reduce frequency of component or speed factor " %
                                   (max_frequency_requested, time_scale, real_fps_requested))

        # Create a new scene for each camera, with specific render resolution
        # Must be done at the end of the builder script, after renaming
        # and before adding 'Scene_Script_Holder'
        self._cfg_camera_scene()

        # Create a new scene for the MORSE_LOGIC (Scene_Script_Holder, CameraFP)
        scene = bpymorse.set_active_scene('S.MORSE_LOGIC')
        scene.game_settings.physics_engine = 'BULLET'
        scene.game_settings.physics_step_sub = self._physics_step_sub
        # set simulation view resolution (4:3)
        scene.render.resolution_x = 800
        scene.render.resolution_y = 600

        # define 'Scene_Script_Holder' as the blender object of Enrivonment
        if not 'Scene_Script_Holder' in bpymorse.get_objects():
            # Add the necessary objects
            base = Component('props', 'basics')

        # Set Scene_Script_Holder as core Environment object
        self.set_blender_object(bpymorse.get_object('Scene_Script_Holder'))
        # Copy properties (for UTM modifier configuration)
        _properties = bpymorse.get_properties(bpymorse.get_object('MORSE.Properties'))
        self.properties(**_properties)

        # Write the configuration of the datastreams, and node configuration
        if not self.multinode_distribution:
            robot_list = None
        else:
            robot_list = self.multinode_distribution.get(self._node_name, [])
            if not isinstance(robot_list, list):
                robot_list = [robot_list]
        Configuration.write_config(robot_list)
        self._write_multinode(self._node_name)

        # Change the Screen material
        if self._display_camera:
            self._set_scren_mat()

        # Write the name of the 'environment file'
        self.properties(environment_file = str(self._environment_file))

        # Default time management
        if 'time_management' not in self._bpy_object.game.properties.keys():
            self.properties(time_management = TimeStrategies.BestEffort)

        if self.fastmode:
            # SINGLETEXTURE support has been removed between 2.69 and
            # 2.70. Handle properly the case where it is not defined
            # anymore.
            try:
                self.set_material_mode('SINGLETEXTURE')
            except TypeError:
                self.set_material_mode('MULTITEXTURE')
            self.set_viewport("WIREFRAME")
        elif not self.is_material_mode_custom:
            # make sure OpenGL shading language shaders (GLSL) is the
            # material mode to use for rendering
            self.set_material_mode('GLSL')

        # Set the unit system to use for button display (in edit mode) to metric
        bpymorse.get_context_scene().unit_settings.system = 'METRIC'
        # Select the type of Framing to Extend,
        # Show the entire viewport in the display window,
        # viewing more horizontally or vertically.
        bpymorse.get_context_scene().game_settings.frame_type = 'EXTEND'
        # Start player with a visible mouse cursor
        bpymorse.get_context_scene().game_settings.show_mouse = True

        # Set the position of the camera
        camera_fp = bpymorse.get_object('CameraFP')
        camera_fp.location = self._camera_location
        camera_fp.rotation_euler = self._camera_rotation
        camera_fp.game.properties['Speed'].value = self._camera_speed
        camera_fp.data.clip_start = self._camera_clip_start
        camera_fp.data.clip_end   = self._camera_clip_end
        camera_fp.data.lens = self._focal_length # set focal length in mm
        # Make CameraFP the active camera
        bpymorse.deselect_all()
        camera_fp.select = True
        bpymorse.get_context_scene().objects.active = camera_fp
        # Set default camera
        bpymorse.get_context_scene().camera = camera_fp
        # Set viewport to Camera
        bpymorse.set_viewport_perspective()

        hud_text = bpymorse.get_object('Keys_text')
        hud_text.scale.y = 0.027 # to fit the HUD_plane

        # Create a cube to compute the dt between two frames
        _dt_name = '__morse_dt_analyser'
        cube = Cube(_dt_name)
        cube.scale = (0.01, 0.01, 0.01)
        cube.location = [0.0, 0.0, -5000.0]
        cube_obj = bpymorse.get_object(_dt_name)
        cube_obj.game.physics_type = 'DYNAMIC'
        cube_obj.hide_render = True
        cube_obj.game.lock_location_z = True

        self._created = True
        # in case we are in edit mode, do not exit on error with CLI
        sys.excepthook = sys.__excepthook__ # Standard Python excepthook

    def set_horizon_color(self, color=(0.05, 0.22, 0.4)):
        """ Set the horizon color

        :sees: `Blender documentation
        <https://docs.blender.org/manual/en/dev/game_engine/world.html#id1>`_


        :param color: (0.0, 0.0, 0.0) < (R, B, G) < (1.0, 1.0, 1.0)
                      default: dark azure (0.05, 0.22, 0.4)
        """
        # Set the color at the horizon to dark azure
        bpymorse.get_context_scene().world.horizon_color = color

    def set_ambient_color(self, color=(0.05, 0.05, 0.05)):
        """ Set the ambient color

        Useful to quickly brighten up or colorize your all scene.

        :sees: `Blender documentation
        <https://docs.blender.org/manual/en/dev/game_engine/world.html#id1>`_

        :param color: (0.0, 0.0, 0.0) < (R, B, G) < (1.0, 1.0, 1.0)
                      default: dark grey (0.05, 0.05, 0.05)
        
        (Modified by William Talbot 23/07/21)
        """
        # Set the ambient color
        bpymorse.get_context_scene().world.ambient_color = color

    def enable_environment_light(self, value=True):
        """ Enables or disables environment light

        (Added by William Talbot 23/07/21)
        """
        if isinstance(value, bool):
            bpymorse.get_context_scene().world.light_settings.use_environment_light = value
    
    def set_environment_light_settings(self, **settings):
        """ Sets the environment light settings for the scene
        """
        if 'environment_energy' in settings:
            bpymorse.get_context_scene().world.light_settings.environment_energy = settings['environment_energy']
        if 'environment_color' in settings:
            bpymorse.get_context_scene().world.light_settings.environment_color = settings['environment_color']
        if 'enable' in settings:
            self.enable_environment_light(settings['enable'])

    def enable_mist(self,value=True):
        """ Enables or disables mist

        See `World/Mist on the Blender Manual
        <http://wiki.blender.org/index.php/Doc:2.6/Manual/World/Mist>`_
        for more information about this particular setting.

        :param value: indicate whether to enable/disable mist
        """
        if isinstance(value, bool):
            bpymorse.get_context_scene().world.mist_settings.use_mist = value

    def set_mist_settings(self, **settings):
        """ Sets the mist settings for the scene

        See `World/Mist on the Blender Manual
        <http://wiki.blender.org/index.php/Doc:2.6/Manual/World/Mist>`_
        for more information about this particular setting.

        Optional arguments need to be specified with identifyer:
        :param enable:     Enables or disables mist
        :param intensity:  Overall minimum intensity of the mist effect in [0,1]
        :param start:      Starting distance of the mist, measured from the camera
        :param depth:      Distance over which the mist effect fades in
        :param falloff:     Type of transition used to fade mist enum in ['QUADRATIC', 'LINEAR', 'INVERSE_QUADRATIC'], default 'QUADRATIC'

        """

        # set the values through bpymorse interface, erronous values are taken care of elsewhere
        if 'falloff' in settings:
            bpymorse.get_context_scene().world.mist_settings.falloff = settings['falloff']
        if 'intensity' in settings:
            bpymorse.get_context_scene().world.mist_settings.intensity = settings['intensity']
        if 'start' in settings:
            bpymorse.get_context_scene().world.mist_settings.start = settings['start']
        if 'depth' in settings:
            bpymorse.get_context_scene().world.mist_settings.depth = settings['depth']
        if 'enable' in settings:
            self.enable_mist(settings['enable'])



    def show_debug_properties(self, value=True):
        """ Display the value of the game-properties marked as debug

        :param value: indicate whether to show or not this information
        """
        if isinstance(value, bool):
            bpymorse.get_context_scene().game_settings.show_debug_properties = value

    def show_framerate(self, value=True):
        """ Display framerate and profile information of the simulation

        :param value: indicate whether to show or not this information
        """
        if isinstance(value, bool):
            bpymorse.get_context_scene().game_settings.show_framerate_profile = value

    def show_physics(self, value=True):
        """ Display of the bounding boxes of objects during the simulation

        :param value: indicate whether to show or not this information
        """
        if isinstance(value, bool):
            bpymorse.get_context_scene().game_settings.show_physics_visualization = value

    def set_gravity(self, gravity=9.81):
        """ Set the gravity for the specific scene

        :param gravity: float, default: 9.81
        """
        if isinstance(gravity, float):
            bpymorse.get_context_scene().game_settings.physics_gravity = gravity
            bpymorse.get_context_scene().gravity = mathutils.Vector((0.0, 0.0, -gravity))

    def set_material_mode(self, material_mode='GLSL'):
        """ Material mode to use for rendering

        - ``SINGLETEXTURE`` Singletexture, Singletexture face materials.
        - ``MULTITEXTURE`` Multitexture, Multitexture materials.
        - ``GLSL`` GLSL, OpenGL shading language shaders.

        :param material_mode: enum in ['SINGLETEXTURE', 'MULTITEXTURE', 'GLSL']
        """
        bpymorse.get_context_scene().game_settings.material_mode = material_mode
        self.is_material_mode_custom = True

    def set_viewport(self, viewport_shade='WIREFRAME', clip_end=1000):
        """ Set the default view mode

        :param viewport_shade: enum in ['BOUNDBOX', 'WIREFRAME', 'SOLID', 'TEXTURED'], default 'WIREFRAME'
        """
        bpymorse.set_viewport(viewport_shade, clip_end)

    def set_auto_start(self, auto_start=True):
        bpymorse.get_context_scene().render.engine = 'BLENDER_GAME'
        bpymorse.get_context_scene().game_settings.use_auto_start = auto_start

    def set_time_strategy(self, strategy):
        """ Choose the time strategy for the current simulation

        :param strategy:  the strategy to choose. Must be one of value
        of :py:class:`morse.builder.TimeStrategies`
        """
        if strategy == TimeStrategies.FixedSimulationStep:
            bpymorse.get_context_scene().game_settings.use_frame_rate = 0
            self.auto_tune_time = False
        elif strategy == TimeStrategies.BestEffort:
            bpymorse.get_context_scene().game_settings.use_frame_rate = 1
        else:
            raise ValueError(strategy)

        self.properties(time_management = strategy)

    def set_time_scale(self, slowdown_by = None, accelerate_by = None):
        """ Slow down or accelerate the simulation relative to real-time
        (default behaviour: real-time simulation) by modifying the *time
        scale* of the simulation.

        :param slowndown_by: factor by which the simulation should be
        slowed down, relative to real-time
        :param accelerate_by: factor by which the simulation should be
        accelerated, relative to real-time
        You must pass only one of these options
        """
        if (not slowdown_by and not accelerate_by) or \
           (slowdown_by and accelerate_by):
            logger.error("You must call set_time_scale with exacltly one of "
                         "the arguments \"slowdown_by\" or \"accelerate_by\"")
            return

        if slowdown_by:
            self.properties(time_scale = 1.0 / slowdown_by)
        else:
            self.properties(time_scale = accelerate_by)

    def use_internal_syncer(self):
        self.auto_tune_time = False
        self.properties(use_internal_syncer = True)
        self.configure_stream_manager('socket', time_sync = True)

    def fullscreen(self, fullscreen=True):
        """ Run the simulation fullscreen

        :param fullscreen: Start player in a new fullscreen display
        :type  fullscreen: Boolean, default: True
        """
        bpymorse.fullscreen(fullscreen)

    def set_debug(self, debug=True):
        """ Set Blender debug mode

        :param debug: set when blender is running in debug mode (started with --debug)
        :type  debug: Boolean, default: True
        """
        bpymorse.set_debug(debug)

    def set_stereo(self, mode='ANAGLYPH', eye_separation=0.1, stereo='STEREO'):
        """ Configure to render image in stereo mode

        (anaglyphs allows to see in 3d with special red-cyan glasses)

        :param mode: Stereographic techniques. enum in ['QUADBUFFERED',
                     'ABOVEBELOW', 'INTERLACED', 'ANAGLYPH', 'SIDEBYSIDE',
                     'VINTERLACE'], default 'ANAGLYPH'
        :param eye_separation: Distance between the eyes. float in [0.01, 5], default 0.1
        :param stereo: enum in ['NONE', 'STEREO', 'DOME'], default 'STEREO'
        """
        bpymorse.get_context_scene().game_settings.stereo = stereo
        bpymorse.get_context_scene().game_settings.stereo_mode = mode
        bpymorse.get_context_scene().game_settings.stereo_eye_separation = eye_separation

    def set_animation_record(self, record=True):
        """ Record the simulation as a Blender animation (F-Curves)

        See the tutorial: `Recording Game Physics to Keyframes
        <http://cgcookie.com/blender/2011/05/10/tip-recording-game-physics-to-keyframes/>`_
        for more information about this particular setting.

        :param record: boolean, default True
        """
        bpymorse.get_context_scene().game_settings.use_animation_record = record

    def set_physics_step_sub(self, step_sub):
        """ Configure the number of physics sub step per physics step.

        Basically, if you increase the step_sub value, your simulation
        will spent more time in computing physics, but it will be more
        precise. The default value is 2.

        See also
        http://www.blender.org/documentation/blender_python_api_2_63_0/bpy.types.SceneGameData.html#bpy.types.SceneGameData.physics_step_sub
        """
        self._physics_step_sub = step_sub

    def configure_multinode(self, protocol='socket',
            server_address='localhost', server_port='65000', distribution=None):
        """ Provide the information necessary for the node to connect to a multi-node server.

        :param protocol: Either 'socket' or 'hla'
        :param server_address: IP address where the multi-node server can be found
        :param server_port: Used only for 'socket' protocol. Currently it should always be 65000
        :param distribution: A Python dictionary. The keys are the names of the
                nodes, and the values are lists with the names of the robots handled by
                each node

        .. code-block:: python

            dala1 = ATRV()
            dala2 = ATRV()

            env = Environment('land-1/trees')
            env.configure_multinode(
                    protocol='socket',
                    server_address='localhost',
                    server_port='65000',
                    distribution={
                        "nodeA": [dala1.name],
                        "nodeB": [dala2.name],
                    })

        """
        self._protocol = protocol
        self._server_address = server_address
        self._server_port = server_port
        if distribution is not None:
            self.multinode_distribution = distribution
        self._multinode_configured = True

    def configure_stream_manager(self, stream_manager, **kwargs):
        if stream_manager in MORSE_DATASTREAM_MODULE:
            stream_manager_classpath = MORSE_DATASTREAM_MODULE[stream_manager]
        else:
            stream_manager_classpath = stream_manager

        Configuration.link_stream_manager_config(stream_manager_classpath, kwargs)

    def configure_service(self, datastream):
        logger.warning("configure_service is deprecated, use add_service instead")
        return self.add_service(datastream)

    def add_service(self, datastream):
        """ Override AbstractComponent method

        Use it to define which ``datastream`` expose the *simulator internals
        services* (*i.e.*, the services used to remotely control the simulator
        behaviour):

        .. code-block:: python

            env = Environement('indoors-1/indoor-1', fastmode = True)
            # Set the simulation management services to be available from ROS:
            env.add_service('ros')

        """
        AbstractComponent.add_service(self, datastream, "simulation")

    def select_display_camera(self, robot_camera):
        """ Select the camera that will be displayed on the HUD Screen object

        :param robot_camera: AbstractComponent reference to the camera desired to be displayed
        """
        self._display_camera = robot_camera

    def _set_scren_mat(self):
        """ Set the material of the Screen object to the same as the one indicated in the _display_camera variable
        """
        camera = None
        screen = bpymorse.get_object('Screen')
        caption = bpymorse.get_object('CameraID_text')
        blender_component = self._display_camera._bpy_object
        # Find the mesh object with a texture called 'ScreenMat'
        for child in blender_component.children:
            if 'CameraMesh' in child.name:
                camera = child
                break
        if not camera:
            logger.warning("BUILDER WARNING: Argument to 'select_display_camera' is not a camera (%s). Camera display will not work" % self._display_camera.name)
            return
        # Find the material with name "ScreenMat"
        for mat in camera.material_slots:
            if "ScreenMat" in mat.name:
                material = mat.material
                break
        logger.debug ("Setting material %s for the Screen" % material)
        screen.active_material = material

        # Make the screen visible when starting the scene
        screen.game.properties['Visible'].value = True

        # Change the text with the name of the camera being displayed
        caption.game.properties['Text'].value = self._display_camera.name

    def save(self, filepath=None, check_existing=False, compress=True):
        """ Save .blend file

        :param filepath: File Path
        :type  filepath: string, (optional, default: current file)
        :param check_existing: Check and warn on overwriting existing files
        :type  check_existing: boolean, (optional, default: False)
        :param compress: Compress, Write compressed .blend file
        :type  compress: boolean, (optional, default: True)
        """
        bpymorse.save(filepath=filepath, check_existing=check_existing,
                compress=compress)

    def set_log_level(self, component, level):
        """
        Set the debug level of the component to the level level.

        :param component: the class name of the component
        :param level: a string representing the level of debug to set

        XXX not persistent (if you save the scene and reload it directly
        from blender, the logger is not set at the right level.
        """

        my_logger = logging.getLogger('morse.' + component)
        my_logger.setLevel(level.upper())

    def set_background_scene(self, scene):
        """
        Set the background scene used by main scene

        :param scene: the name of the scene to use in background
        """
        bpymorse.get_context_scene().background_set = bpymorse.get_scene(scene)

    def simulator_frequency(self, base_frequency=60, logic_step_max=20, physics_step_max=20):
        """ Tune the frequency of the simulation

        :param base_frequency: Nominal number of game frames per second
            (physics fixed timestep = 1/ (base_frequency * time_scale),
            independently of actual frame rate)
        :type base_frequency: default 60
        :param logic_step_max: Maximum number of logic frame per game frame if
            graphics slows down the game, higher value allows better
            synchronization with physics
        :type logic_step_max: default value : 20
        :param physics_step_max: Maximum number of physics step per game frame
            if graphics slows down the game, higher value allows physics to keep
            up with realtime
        :type physics_step_max: default value : 20

        usage::

            env.simulator_frequency(120, 5, 5)

        .. note:: It is recommended to use the same value for
           logic_step_max and physics_step_max
        """
        scene = bpymorse.get_context_scene()
        scene.game_settings.fps = base_frequency
        scene.game_settings.logic_step_max = logic_step_max
        scene.game_settings.physics_step_max = physics_step_max
        self.auto_tune_time = False

    def use_vsync(self, vsync):
        """  Configure vsync parameter for the current scene

        :param vsync: should be one ['ON', 'OFF', 'ADAPTIVE']
        """
        bpymorse.get_context_scene().game_settings.vsync = vsync

    def use_relative_time(self, relative_time):
        """ Configure if Morse should exports relative time (time since
        Morse start) or "absolute" start (time since Epoch)
        """
        self.properties(use_relative_time = relative_time)

    def use_display_lists(self, display_lists=True):
        """ Use display lists to speed up rendering by keeping geometry on the GPU """
        bpymorse.get_context_scene().game_settings.use_display_lists = display_lists

    def use_material_caching(self, material_caching=True):
        """ Cache materials in the converter (this is faster, but can cause problems
        with older Singletexture and Multitexture games).
        
        WARNING: Turning off material caching may hang morse."""
        bpymorse.get_context_scene().game_settings.use_material_caching = material_caching

    def __del__(self):
        """ Call the create method if the user has not explicitly called it """
        if not self._created:
            self.create()

