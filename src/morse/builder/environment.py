import logging; logger = logging.getLogger("morsebuilder." + __name__)
import os
import json
from morse.builder.morsebuilder import *

class Environment(Component):
    """ Class to configure the general environment of the simulation

    It handles the background environment in which your robots are simulated,
    general properties of the simulation, the default location and orientation
    of the camera, the Blender Game Engine settings, configure the parameters
    for the multi-node simulation and also writes the 'component_config.py' file.
    """
    multinode_distribution = dict()

    def __init__(self, filename, fastmode = False):
        """
        :param fastmode: (default: False) if True, disable most visual
                         effects (like lights...) to get the fastest
                         running simulation.  Useful for unit-tests for
                         instance, or in simulations where realistic
                         environment texturing is not required (*e.g.*,
                         no video camera)

        """
        Component.__init__(self, 'environments', filename)
        AbstractComponent.components.remove(self) # remove myself from the list of components to ensure my destructor is called

        # Rename the components according to their variable names
        self._rename_components()

        self.fastmode = fastmode

        self._created = False
        self._camera_location = [5, -5, 5]
        self._camera_rotation = [0.7854, 0, 0.7854]
        self._environment_file = filename
        self._multinode_configured = False
        self._display_camera = None
        self.is_material_mode_custom = False

        # define 'Scene_Script_Holder' as the blender object of Enrivonment
        if not 'Scene_Script_Holder' in bpymorse.get_objects():
            # Add the necessary objects
            base = Component('props', 'basics')
        self.set_blender_object(bpymorse.get_object('Scene_Script_Holder'))
        # Write the name of the 'environment file'

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
        cfg.write('node_config = ' + json.dumps(node_config, indent=1) )
        cfg.write('\n')

    def _rename_components(self):
        """ Rename Blender objects after the variable name used
        in the Builder script and there hierarchy.

        If a name is already set (with 'obj.name=...'), it is used as it,
        and only the hierarchy is added to the name.
        """

        import inspect
        try:
            frame = inspect.currentframe()
            builderscript_frame = inspect.getouterframes(frame)[2][0] # parent frame

            for name, component in builderscript_frame.f_locals.items():
                if isinstance(component, AbstractComponent):

                    if hasattr(component, "parent"):
                        continue

                    if not component.basename: # do automatic renaming only if a name is not already manually set
                        component.basename = name

                    def renametree(cmpt, fqn):
                        if not cmpt.basename:
                            raise SyntaxError("You need to assign the component of type %s to a variable" %
                                             cmpt)
                        fqn.append(cmpt.basename)
                        new_name = '.'.join(fqn)
                        Configuration.update_name(cmpt.name, new_name)
                        cmpt._bpy_object.name = '.'.join(fqn)
                        for child in cmpt.children:
                            renametree(child, fqn[:])

                    renametree(component, [])
        finally:
            del builderscript_frame
            del frame

    def place_camera(self, location):
        """ Set the location of the default camera.

        :param location: list with the new 3D coordinates for the camera.

        .. code-block:: python

            env.place_camera([10.0, -10.0, 3.0])

        """
        self._camera_location = location

    def aim_camera(self, rotation):
        """ Set the orientation of the default camera

        :param rotation: list with an euler rotation for the camera.

        .. code-block:: python

            env.aim_camera([1.3300, 0, 0.7854])

        """
        self._camera_rotation = rotation

    def set_camera_clip(self, clip_start=0.1, clip_end=100):
        """ Set the simulator's Camera near and far clipping distance

        :param clip_start: Camera near clipping distance, float in meters (default 0.1)
        :param clip_end: Camera far clipping distance, float in meters (default 100)
        """
        camera_fp = bpymorse.get_object('CameraFP')
        # camera_fp.data holds the bpy.types.Camera instance
        camera_fp.data.clip_start = clip_start
        camera_fp.data.clip_end = clip_end

    def create(self, name=None):
        """ Generate the scene configuration and insert necessary objects

        Should always be called at the very end of the Builder script. It will
        finalise the building process and write the configuration files.
        """
        try:
            # Invoke special methods of component that must take place *after* renaming
            for component in AbstractComponent.components:
                if hasattr(component, "after_renaming"):
                    component.after_renaming()



            # Default node name
            if name == None:
                try:
                    name = os.environ["MORSE_NODE"]
                except KeyError:
                    name = os.uname()[1]
            # Write the configuration of the datastreams, and node configuration
            Configuration.write_config()
            self._write_multinode(name)

            # Change the Screen material
            if self._display_camera:
                self._set_scren_mat()

            self.properties(environment_file = str(self._environment_file))
            # Set the position of the camera
            camera_fp = bpymorse.get_object('CameraFP')
            camera_fp.location = self._camera_location
            camera_fp.rotation_euler = self._camera_rotation

            if self.fastmode:
                self.set_material_mode('SINGLETEXTURE')
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

            # Make CameraFP the active camera
            bpymorse.deselect_all()
            camera_fp.select = True
            bpymorse.get_context_scene().objects.active = camera_fp
            # Set default camera
            bpymorse.get_context_scene().camera = camera_fp

            self._created = True
        except BaseException:
            logger.error("Your MORSE Builder script is invalid!")
            import traceback
            traceback.print_exc()
            os._exit(-1)

    def set_horizon_color(self, color=(0.05, 0.22, 0.4)):
        """ Set the horizon color

        See `World/Background on the Blender Manual
        <http://wiki.blender.org/index.php/Doc:2.6/Manual/World/Background>`_
        for more information about this particular setting.

        :param color: (0.0, 0.0, 0.0) < (R, B, G) < (1.0, 1.0, 1.0)
                      default: dark azure (0.05, 0.22, 0.4)
        """
        # Set the color at the horizon to dark azure
        bpymorse.get_context_scene().world.horizon_color = color

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

    def set_material_mode(self, material_mode='GLSL'):
        """ Material mode to use for rendering

        - ``SINGLETEXTURE`` Singletexture, Singletexture face materials.
        - ``MULTITEXTURE`` Multitexture, Multitexture materials.
        - ``GLSL`` GLSL, OpenGL shading language shaders.

        :param material_mode: enum in ['SINGLETEXTURE', 'MULTITEXTURE', 'GLSL']
        """
        bpymorse.get_context_scene().game_settings.material_mode = material_mode
        self.is_material_mode_custom = True

    def set_viewport(self, viewport_shade='WIREFRAME'):
        """ Set the default view mode

        :param viewport_shade: enum in ['BOUNDBOX', 'WIREFRAME', 'SOLID', 'TEXTURED'], default 'WIREFRAME'
        """
        for area in bpymorse.get_context_window().screen.areas:
            if area.type == 'VIEW_3D':
                for space in area.spaces:
                    if space.type == 'VIEW_3D':
                        space.viewport_shade = viewport_shade

    def set_auto_start(self, auto_start=True):
        bpymorse.get_context_scene().render.engine = 'BLENDER_GAME'
        bpymorse.get_context_scene().game_settings.use_auto_start = auto_start

    def set_debug(self, debug=True):
        """ Set the debug bit in blender """
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
        if distribution != None:
            self.multinode_distribution = distribution
        self._multinode_configured = True

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
            env.configure_service('ros')

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

    def __del__(self):
        """ Call the create method if the user has not explicitly called it """
        if not self._created:
            self.create()
