import logging; logger = logging.getLogger("morsebuilder." + __name__)
import os
import json
import math
from morse.builder.abstractcomponent import *

"""
Morse Builder API

To test this module you can c/p the following code in Blender Python console::

.. code-block:: python

    import sys
    sys.path.append("/usr/local/lib/python3/dist-packages")
    from morse.builder import *
    atrv=Robot("atrv")

The string passed to the differents Components Classes must be an existing
.blend file-name, ie. for ``Robot("atrv")`` the file ``atrv.blend`` must exists
in the folder ``MORSE_COMPONENTS/robots/``.
"""

class PassiveObject(AbstractComponent):
    """ Allows to import any Blender object to the scene.
    """

    def __init__(self, filename="props/objects", prefix=None, keep_pose=False):
        """
        :param filename: The Blender file to load. Path can be absolute
                         or if no extension relative to MORSE assets'
                         installation path (typically, $PREFIX/share/morse/data)
        :param prefix: (optional) the prefix of the objects to load in the
                       Blender file. If not set, all objects present in the file
                       are loaded. If set, all objects **prefixed** by this
                       name are imported.
        :param keep_pose: If set, the object pose (translation and rotation)
                        in the Blender file is kept. Else, the object
                        own center is placed at origin and all rotation are
                        reset.
        :return: a new AbstractComponent instance.
        """
        AbstractComponent.__init__(self, filename=filename)

        logger.info("Importing the following passive object(s): %s" % (prefix))

        imported_objects = self.append_meshes(prefix=prefix)
        # Here we use the fact that after appending, Blender select the objects
        # and the root (parent) object first ( [0] )
        self.set_blender_object(imported_objects[0])

        if not keep_pose:
            self.location = (0.0, 0.0, 0.0)
            self.rotation_euler = (0.0, 0.0, 0.0)

    def setgraspable(self):
        """
        Makes an object graspable to the human avatar by adding a NEAR collision
        sensor to the object.

        This function also set the object to be an active game object (property
        'Object' set to true), and set the object label to the Blender object
        name (if not already set).
        """
        obj = self._bpy_object

        if not "Label" in obj.game.properties:
            self.properties(Object = True, Graspable = True, Label = obj.name)
        else:
            self.properties(Object = True, Graspable = True)

        # Add collision sensor for object placement
        if not 'Collision' in obj.game.sensors:
            bpymorse.add_sensor(type = 'NEAR')
            sens = obj.game.sensors[-1]
            sens.name = 'Collision'
            sens.distance = 0.05
            sens.reset_distance = 0.075
            bpymorse.add_controller()
            contr = obj.game.controllers[-1]
            contr.link(sensor = sens)

class Component(AbstractComponent):
    """ Append a morse-component to the scene

    cf. `bpy.ops.wm.link_append` and `bpy.data.libraries.load`
    """
    def __init__(self, category='', filename='', make_morseable=True):
        """ Initialize a MORSE component

        :param category: The category of the component (folder in
            MORSE_COMPONENTS)
        :param filename: The name of the component (file in
            MORSE_COMPONENTS/category/name.blend) If ends with '.blend',
            append the objects from the Blender file.
        :param make_morseable: If the component has no property for the
            simulation, append default Morse ones. See self.morseable()
        """
        AbstractComponent.__init__(self, filename=filename, category=category)
        imported_objects = self.append_meshes()
        # Here we use the fact that after appending, Blender select the objects
        # and the root (parent) object first ( [0] )
        self.set_blender_object(imported_objects[0])
        # If the object has no MORSE logic, add default one
        if make_morseable and category in ['sensors', 'actuators', 'robots'] \
                and not self.is_morseable():
            self.morseable()


class Armature(AbstractComponent):
    def __init__(self, objectname, filename='armature'):
        """ Initialize an Armature

        :param objectname: Armature name
        :param filename: for datastream configuration, default 'armature'
        """
        AbstractComponent.__init__(self, filename=filename)
        self.set_blender_object(bpymorse.get_object(objectname))


class Robot(Component):
    def __init__(self, filename):
        Component.__init__(self, 'robots', filename)

    def add_default_interface(self, stream):
        for child in self.children:
            if child.is_morseable():
                child.add_interface(stream)

    def make_external(self):
        self._bpy_object.game.properties['Robot_Tag'].name = 'External_Robot_Tag'
#    def remove_wheels(self):
#        wheels = [child for child in self._bpy_object.children if \
#                  child.name.lower().startswith("wheel")]
#        for wheel in wheels:
#            bpymorse.deselect_all()
#            bpy.ops.object.select_pattern(pattern=wheel.name, extend=False)
#            bpy.ops.object.parent_clear(type='CLEAR_KEEP_TRANSFORM')
#    def __del__(self):
#        """ Call the remove_wheels method if the robot is a Bullet Vehicle """
#        # HasSuspension game property is used for Bullet Vehicle
#        if "HasSuspension" in self._bpy_object.game.properties:
#            self.remove_wheels()


class WheeledRobot(Robot):
    def __init__(self, filename):
        Robot.__init__(self, filename)

    def unparent_wheels(self):
        """ Make the wheels orphans, but keep the transormation applied to
            the parent robot """
        # Force Blender to update the transformation matrices of objects
        bpymorse.get_context_scene().update()
        wheels = [child for child in self._bpy_object.children if \
                  "wheel" in child.name.lower()]
        import mathutils
        for wheel in wheels:
            # Make a copy of the current transformation matrix
            transformation = mathutils.Matrix(wheel.matrix_world)
            wheel.parent = None
            wheel.matrix_world = transformation
            # This method should be easier, but does not seem to work
            #  because of an incorrect context error
            #bpy.ops.object.parent_clear(type='CLEAR_KEEP_TRANSFORM')

    def append(self, obj):
        """ Add a child to the current object,
        Overload the append method of AbstractObject
        eg: robot.append(sensor), will set the robot parent of the sensor.
        """
        # Correct the rotation of the object
        old = obj._bpy_object.rotation_euler
        obj._bpy_object.rotation_euler = (old[0], old[1], old[2]+math.pi/2)

        # Switch the values of X and Y location
        tmp_x = obj._bpy_object.location[0]
        obj._bpy_object.location[0] = -obj._bpy_object.location[1]
        obj._bpy_object.location[1] = tmp_x

        Robot.append(self, obj, 2)


class Sensor(Component):
    def __init__(self, filename):
        Component.__init__(self, 'sensors', filename)

    # @deprecated
    def create_sick_arc(self):
        """ Create an arc for use with the SICK sensor

        The arc is created using the parameters in the Sick Empty.
        'resolution and 'scan_window' are used to determine how many points
        will be added to the arc.
        """
        logger.warning("DEPRECATED : Sensor.create_sick_arc is deprecated, " + \
                       "use LaserSensorWithArc.create_laser_arc instead")

        scene = bpymorse.get_context_scene()

        sick_obj = self._bpy_object

        material = None
        # Get the mesh and the "RayMat" material for the arc
        for mat in bpymorse.get_materials():
            if "RayMat" in mat.name:
                material = mat.material
                break

        # Delete previously created arc
        for child in sick_obj.children:
            if child.name.startswith("Arc_"):
                scene.objects.unlink( child )

        # Read the parameters to create the arc
        properties = sick_obj.game.properties
        resolution = properties['resolution'].value
        window = properties['scan_window'].value
        # Parameters for multi layer sensors
        try:
            layers = properties['layers'].value
            layer_separation = properties['layer_separation'].value
            layer_offset = properties['layer_offset'].value
        except KeyError as detail:
            layers = 1
            layer_separation = 0.0
            layer_offset = 0.0
        logger.debug ("Creating %d arc(s) of %.2f degrees, resolution %.2f" % (layers, window, resolution))
        mesh = bpymorse.new_mesh( "ArcMesh" )
        # Add the center vertex to the list of vertices
        verts = [ [0.0, 0.0, 0.0] ]
        faces = []
        vertex_index = 0

        # Set the vertical angle, in case of multiple layers
        if layers > 1:
            v_angle = layer_separation * (layers-1) / 2.0
        else:
            v_angle = 0.0

        # Initialise the parameters for every layer
        for layer_index in range(layers):
            start_angle = window / 2.0
            end_angle = -window / 2.0
            # Offset the consecutive layers
            if (layer_index % 2) == 0:
                start_angle += layer_offset
                end_angle += layer_offset
            logger.debug ("Arc from %.2f to %.2f" % (start_angle, end_angle))
            logger.debug ("Vertical angle: %.2f" % v_angle)
            arc_angle = start_angle

            # Create all the vertices and faces in a layer
            while arc_angle >= end_angle:
                # Compute the coordinates of the new vertex
                new_vertex = [ math.cos(math.radians(arc_angle)), math.sin(math.radians(arc_angle)), math.sin(math.radians(v_angle)) ]
                verts.append(new_vertex)
                vertex_index = vertex_index + 1
                # Add the faces after inserting the 2nd vertex
                if arc_angle < start_angle:
                    faces.append([0, vertex_index-1, vertex_index])
                # Increment the angle by the resolution
                arc_angle = arc_angle - resolution

            v_angle -= layer_separation

        mesh.from_pydata( verts, [], faces )
        mesh.update()
        # Compose the name of the arc
        arc_name = "Arc_%d" % window
        arc = bpymorse.new_object( arc_name, mesh )
        arc.data = mesh
        # Remove collision detection for the object
        arc.game.physics_type = 'NO_COLLISION'
        # Set the material of the arc
        arc.active_material = material
        # Link the new object in the scene
        scene.objects.link( arc )
        # Set the parent to be the Sick Empty
        arc.parent = sick_obj


class Actuator(Component):
    def __init__(self, filename):
        Component.__init__(self, 'actuators', filename)


class Environment(Component):
    """ Class to configure the general environment of the simulation
    It handles the scenario file, general properties of the simulation,
    the default location and orientation of the camera, the Blender GE settings
    and also writes the 'component_config.py' file.
    """
    multinode_distribution = dict()

    def __init__(self, filename, fastmode = False):
        """
        :param fastmode: (default: False) if True, disable most visual effects (like lights...) to
        get the fastest running simulation. Useful for unit-tests for instance, or in simulations
        where realistic environment texturing is not required (eg, no video camera)
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
        if not 'multinode_config.py' in bpymorse.get_texts.keys():
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
        """ Store the position that will be givent to the default camera
        Expected argument is a list with the desired position for the camera
        """
        self._camera_location = location

    def aim_camera(self, rotation):
        """ Store the orientation that will be givent to the default camera
        Expected argument is a list with the desired orientation for the camera
        """
        self._camera_rotation = rotation

    def create(self, name=None):
        """ Generate the scene configuration and insert necessary objects
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

            if not self.fastmode:
                # make sure OpenGL shading language shaders (GLSL) is the
                # material mode to use for rendering
                bpymorse.get_context_scene().game_settings.material_mode = 'GLSL'
            else:
                bpymorse.get_context_scene().game_settings.material_mode = 'SINGLETEXTURE'
                self.set_viewport("WIREFRAME")

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
        :param color: (0.0, 0.0, 0.0) < (R, B, G) < (1.0, 1.0, 1.0)
                      default: dark azure (0.05, 0.22, 0.4)
        """
        # Set the color at the horizon to dark azure
        bpymorse.get_context_scene().world.horizon_color = color

    def show_debug_properties(self, value=True):
        if isinstance(value, bool):
            bpymorse.get_context_scene().game_settings.show_debug_properties = value

    def show_framerate(self, value=True):
        if isinstance(value, bool):
            bpymorse.get_context_scene().game_settings.show_framerate_profile = value

    def show_physics(self, value=True):
        if isinstance(value, bool):
            bpymorse.get_context_scene().game_settings.show_physics_visualization = value

    def set_gravity(self, gravity=9.81):
        if isinstance(gravity, float):
            bpymorse.get_context_scene().game_settings.physics_gravity = gravity

    def set_viewport(self, viewport_shade='WIREFRAME'):
        """ set_viewport
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
        bpymorse.set_debug(debug)

    def set_stereo(self, mode='ANAGLYPH', eye_separation=0.1, stereo='STEREO'):
        """ set_stereo
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
        """ Record animation to F-Curves, so you can render it later
        :param record: boolean, default True
        """
        bpymorse.get_context_scene().game_settings.use_animation_record = record

    def configure_multinode(self, protocol='socket',
            server_address='localhost', server_port='65000', distribution=None):
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
        """ override AbstractComponent method """
        AbstractComponent.add_service(self, datastream, "simulation")

    def select_display_camera(self, robot_camera):
        """ Select the camera that will be displayed on the Screen object
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

    def profile(self, component):
        """ Watch the average time used by the :param component: during the
        simulation, in percent.
        """
        if component._category is not 'sensors':
            logger.warning("currently supports only sensors")
        prop = component._property_new("profile", "0 %")
        prop.show_debug = True
        prop = component._property_new("profile::action", "0 %")
        prop.show_debug = True
        prop = component._property_new("profile::modifiers", "0 %")
        prop.show_debug = True
        prop = component._property_new("profile::datastreams", "0 %")
        prop.show_debug = True
        self.show_debug_properties()

    def save(self, filepath=None, check_existing=False):
        """ Save .blend file

        :param filepath: (string, (optional, default: current file)) File Path
        :param check_existing: (boolean, (optional, default: False))
                               Check and warn on overwriting existing files
        """
        bpymorse.save(filepath=filepath, check_existing=check_existing)

    def __del__(self):
        """ Call the create method if the user has not explicitly called it """
        if not self._created:
            self.create()
