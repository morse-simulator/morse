import logging; logger = logging.getLogger("morsebuilder." + __name__)
import os
import bpy
import json
from morse.core.exceptions import MorseError
from morse.core.exceptions import MorseBuilderNoComponentError
from morse.builder.abstractcomponent import *

"""
Morse Builder API

To test this module you can c/p the following code in Blender Python console::

.. code-block:: python
    import sys
    sys.path.append("/usr/local/lib/python3/dist-packages")
    from morse.builder.morsebuilder import *
    atrv=Robot("atrv")

The string passed to the differents Components Classes must be an existing 
.blend file-name, ie. for ``Robot("atrv")`` the file ``atrv.blend`` must exists 
in the folder ``MORSE_COMPONENTS/robots/``.
"""

class PassiveObject(AbstractComponent):
    """ Allows to import any Blender object to the scene.
    """

    def __init__(self, file, prefix = None, keep_pose = False):
        """
        :param blenderfile: The Blender file to load. Path can be absolute
                           or relative to MORSE assets' installation path
                           (typically, $PREFIX/share/morse/data)
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
        AbstractComponent.__init__(self)
        if os.path.exists(file):
            filepath = file
        else:
            filepath = os.path.join(MORSE_COMPONENTS, file)
            if not os.path.exists(filepath):
                logger.error("Blender file %s for external asset import can \
                         not be found.\n Either provide an absolute path, or \
                         a path relative to MORSE \nassets directory (typically \
                         $PREFIX/share/morse/data)" % (file))

        with bpy.data.libraries.load(filepath) as (src, _):
            if prefix:
                objlist = [{'name':obj} for obj in src.objects if obj.startswith(prefix)]
            else:
                try:
                    objlist = [{'name':obj} for obj in src.objects]
                except UnicodeDecodeError as detail:
                    logger.error("Unable to open file '%s'. Exception: %s" \
                                 % (filepath, detail))

        logger.info("Importing the following passive object(s): %s" % (objlist))

        bpy.ops.object.select_all(action='DESELECT')
        bpy.ops.wm.link_append(directory=filepath + '/Object/', link=False, 
                autoselect=True, files=objlist)
        # here we use the fact that after appending, Blender select the objects 
        # and the root (parent) object first ( [0] )
        self._blendobj = bpy.context.selected_objects[0]

        if not keep_pose:
            self._blendobj.location = (0.0, 0.0, 0.0)
            self._blendobj.rotation_euler = (0.0, 0.0, 0.0)
        
    def setgraspable(self):
        """
        Makes an object graspable to the human avatar by adding a NEAR collision
        sensor to the object.
        
        This function also set the object to be an active game object (property 
        'Object' set to true), and set the object label to the Blender object 
        name (if not already set).        
        """
        obj = self._blendobj
        
        if not "Label" in obj.game.properties:
            self.properties(Object = True, Graspable = True, Label = obj.name)
        else:
            self.properties(Object = True, Graspable = True)        
        
        # Add collision sensor for object placement
        if not 'Collision' in obj.game.sensors:
            bpy.ops.logic.sensor_add(type = 'NEAR')
            sens = obj.game.sensors[-1]
            sens.name = 'Collision'
            sens.distance = 0.05
            sens.reset_distance = 0.075
            bpy.ops.logic.controller_add()
            contr = obj.game.controllers[-1]
            contr.link(sensor = sens)

class Human(AbstractComponent):
    """ Append a human model to the scene.

    The human model currently available in MORSE comes with its
    own subjective camera and several features for object manipulation.

    It also exposes a :doc:`human posture component <morse/user/sensors/human_posture>`
    that can be accessed by the ``armature`` member.

    Usage example:

    .. code-block:: python
       #! /usr/bin/env morseexec

       from morse.builder.morsebuilder import *

       human = Human()
       human.translate(x=5.5, y=-3.2, z=0.0)
       human.rotate(z=-3.0)

       human.armature.configure_mw('pocolibs',
                        ['Pocolibs',
                         'export_posture',
                         'morse/middleware/pocolibs/sensors/human_posture',
                         'human_posture'])

    Currently, only one human per simulation is supported.
    """
    def __init__(self):
        AbstractComponent.__init__(self)
        filepath = os.path.join(MORSE_COMPONENTS, 'robots', 'human.blend')

        with bpy.data.libraries.load(filepath) as (src, _):
            try:
                objlist = [{'name':obj} for obj in src.objects]
            except UnicodeDecodeError as detail:
                logger.error("Unable to open file '%s'. Exception: %s" % \
                             (filepath, detail))

        bpy.ops.object.select_all(action='DESELECT')
        bpy.ops.wm.link_append(directory=filepath + '/Object/', link=False, 
                autoselect=True, files=objlist)
        self._blendname = "Human" # for middleware dictionary
        self._blendobj = self._get_selected("Human")

        self.armature = None

        try:
            obj = self._get_selected("HumanArmature")
            self.armature = AbstractComponent(obj, "human_posture")
        except KeyError:
            logger.error("Could not find the human armature! (I was looking " +\
                         "for an object called 'HumanArmature' in the 'Human'" +\
                         " children). I won't be able to export the human pose" +\
                         " to any middleware.")

        # fix for Blender 2.6 Animations
        if bpy.app.version > (2,59,0):
            if obj:
                hips = self._get_selected("Hips_Empty")
                i = 0
                for act in hips.game.actuators:
                    act.layer = i
                    i = i + 1

                i = 0
                for act in obj.game.actuators:
                    act.layer = i
                    i = i + 1


class Component(AbstractComponent):
    """ Append a morse-component to the scene

    cf. `bpy.ops.wm.link_append` and `bpy.data.libraries.load`
    """
    def __init__(self, category='', name='', make_morseable=True):
        """ Initialize a MORSE component

        :param category: The category of the component (folder in 
            MORSE_COMPONENTS)
        :param name: The name of the component (file in 
            MORSE_COMPONENTS/category/name.blend) If ends with '.blend', 
            append the objects from the Blender file.
        :param make_morseable: If the component has no property for the 
            simulation, append default Morse ones. See self.morseable()
        """
        AbstractComponent.__init__(self, name=name)
        if name.endswith('.blend'):
            filepath = os.path.abspath(name) # external blend file
        else:
            filepath = os.path.join(MORSE_COMPONENTS, category, name + '.blend')

        try: 
            with bpy.data.libraries.load(filepath) as (src, _):
                try:
                    objlist = [{'name':obj} for obj in src.objects]
                except UnicodeDecodeError as detail:
                    logger.error("Unable to open file '%s'. Exception: %s" % \
                                 (filepath, detail))
        except IOError as detail:
            logger.error(detail)
            raise MorseBuilderNoComponentError("Component not found")

        bpy.ops.object.select_all(action='DESELECT')
        bpy.ops.wm.link_append(directory=filepath + '/Object/', link=False, 
                autoselect=True, files=objlist)
        # here we use the fact that after appending, Blender select the objects 
        # and the root (parent) object first ( [0] )
        self._blendobj = bpy.context.selected_objects[0]
        self._category = category
        if make_morseable and category in ['sensors', 'actuators', 'robots'] \
                and not self.is_morseable():
            self.morseable()
    def is_morseable(self):
        return 'Class' in self._blendobj.game.properties
    def morseable(self, calling_module=None):
        """ Make this component simulable in MORSE

        :param calling_module: Module called each simulation cycle.
            enum in ['calling.sensor_action', 'calling.actuator_action', 
                    'calling.robot_action']
        """
        obj = self._blendobj
        if not calling_module:
            calling_module = 'calling.'+self._category[:-1]+'_action'
        # add default class to this component
        if calling_module == 'calling.robot_action':
            self.properties(Robot_Tag = True, Path = 'morse/core/robot', \
                Class = 'MorseRobotClass')
        elif calling_module == 'calling.sensor_action':
            self.properties(Component_Tag = True, Path = 'morse/core/sensor', \
                Class = 'MorseSensorClass')
        elif calling_module == 'calling.actuator_action':
            self.properties(Component_Tag = True, Path = 'morse/core/actuator',\
                Class = 'MorseActuatorClass')
        else:
            logger.warning(self.name + ": unknown category: " + calling_module)

        # add Game Logic sensor and controller to simulate the component
        bpy.ops.object.select_all(action = 'DESELECT')
        bpy.ops.object.select_name(name = obj.name)
        bpy.ops.logic.sensor_add() # default is Always sensor
        sensor = obj.game.sensors[-1]
        sensor.use_pulse_true_level = True
        bpy.ops.logic.controller_add(type='PYTHON')
        controller = obj.game.controllers[-1]
        controller.mode = 'MODULE'
        controller.module = calling_module
        controller.link(sensor = sensor)
    def frequency(self, delay=0):
        """ Set the frequency delay for the call of the Python module

        :param delay: (int) Delay between repeated pulses 
            (in logic tics, 0 = no delay)
        """
        sensors = [s for s in self._blendobj.game.sensors if s.type == 'ALWAYS']
        if len(sensors) > 1:
            logger.warning(self.name + " has too many Game Logic sensors to "+\
                    "tune its frequency, change it through Blender")
        sensors[0].frequency = delay


class Robot(Component):
    def __init__(self, name):
        Component.__init__(self, 'robots', name)
    def make_external(self):
        self._blendobj.game.properties['Robot_Tag'].name = 'External_Robot_Tag'


class WheeledRobot(Robot):
    def __init__(self, name):
        Robot.__init__(self, name)
        self._wheels = []
        #self.unparent_wheels()

    def unparent_wheels (self):
        """ Make the wheels orphans, but keep the transormation applied to
            the parent robot """
        # Force Blender to update the transformation matrices of objects
        bpy.data.scenes['Scene'].update()
        children = self._blendobj.children
        self._wheels = [child for child in children if "wheel" in child.name.lower()]
        import mathutils
        for wheel in self._wheels:
            # Make a copy of the current transformation matrix
            transformation = mathutils.Matrix(wheel.matrix_world)
            wheel.parent = None
            wheel.matrix_world = transformation

    def append(self, obj):
        """ Add a child to the current object,
        Overload the append method of AbstractObject
        eg: robot.append(sensor), will set the robot parent of the sensor.
        """
        import math
        # Correct the rotation of the object
        old = obj._blendobj.rotation_euler
        obj._blendobj.rotation_euler = (old[0], old[1], old[2]+math.pi/2)

        # Switch the values of X and Y location
        tmp_x = obj._blendobj.location[0]
        obj._blendobj.location[0] = -obj._blendobj.location[1]
        obj._blendobj.location[1] = tmp_x

        obj._blendobj.parent = self._blendobj


class Sensor(Component):
    def __init__(self, name):
        Component.__init__(self, 'sensors', name)


class Actuator(Component):
    def __init__(self, name):
        Component.__init__(self, 'actuators', name)


class Environment(AbstractComponent):
    """ Class to configure the general environment of the simulation
    It handles the scenario file, general properties of the simulation,
    the default location and orientation of the camera, the Blender GE settings
    and also writes the 'component_config.py' file.
    """

    multinode_distribution = dict()

    def __init__(self, name=None):
        if name:
            Component.__init__(self, 'environments', name)
        else:
            AbstractComponent.__init__(self)
        self._created = False
        self._camera_location = [5, -5, 5]
        self._camera_rotation = [0.7854, 0, 0.7854]
        self._environment_file = name
        self._multinode_configured = False
        self._display_camera = None
        
        # define 'Scene_Script_Holder' as the blender object of Enrivonment 
        if not 'Scene_Script_Holder' in bpy.data.objects:
            # Add the necessary objects
            base = Component('props', 'basics')
        self._blendobj = bpy.data.objects['Scene_Script_Holder']
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
        for obj in bpy.data.objects:
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

        """ Write the 'multinode_config.py' script """
        node_config = { 'protocol': self._protocol,
                        'node_name': node_name,
                        'server_address': self._server_address,
                        'server_port': self._server_port,}
        # Create the config file if it does not exist
        if not 'multinode_config.py' in bpy.data.texts.keys():
            bpy.ops.text.new()
            bpy.data.texts[-1].name = 'multinode_config.py'
        cfg = bpy.data.texts['multinode_config.py']
        cfg.clear()
        cfg.write('node_config = ' + json.dumps(node_config, indent=1) )
        cfg.write('\n')

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
        # Default node name
        if name == None:
            try:
                name = os.environ["MORSE_NODE"]
            except KeyError:
                name = os.uname()[1]
        # Write the configuration of the middlewares, and node configuration
        Configuration().write_config()
        self._write_multinode(name)

        # Change the Screen material
        if self._display_camera:
            self._set_scren_mat()
        
        self.properties(environment_file = str(self._environment_file))
        # Set the position of the camera
        camera_fp = bpy.data.objects['CameraFP']
        camera_fp.location = self._camera_location
        camera_fp.rotation_euler = self._camera_rotation
        # Make CameraFP the active camera
        bpy.ops.object.select_all(action = 'DESELECT')
        bpy.ops.object.select_name(name = 'CameraFP')
        # make sure OpenGL shading language shaders (GLSL) is the 
        # material mode to use for rendering
        bpy.context.scene.game_settings.material_mode = 'GLSL'
        # Set the unit system to use for button display (in edit mode) to metric
        bpy.context.scene.unit_settings.system = 'METRIC'
        # Select the type of Framing to Extend, 
        # Show the entire viewport in the display window, 
        # viewing more horizontally or vertically.
        bpy.context.scene.game_settings.frame_type = 'EXTEND'
        # Start player with a visible mouse cursor
        bpy.context.scene.game_settings.show_mouse = True
        # Set default camera
        bpy.context.scene.camera = camera_fp
        self._created = True

    def set_horizon_color(self, color=(0.05, 0.22, 0.4)):
        """ Set the horizon color
        :param color: (0.0, 0.0, 0.0) < (R, B, G) < (1.0, 1.0, 1.0) 
                      default: dark azure (0.05, 0.22, 0.4)
        """
        # Set the color at the horizon to dark azure
        bpy.context.scene.world.horizon_color = color

    def show_debug_properties(self, value=True):
        if isinstance(value, bool):
            bpy.data.scenes[0].game_settings.show_debug_properties = value

    def show_framerate(self, value=True):
        if isinstance(value, bool):
            bpy.data.scenes[0].game_settings.show_framerate_profile = value

    def show_physics(self, value=True):
        if isinstance(value, bool):
            bpy.data.scenes[0].game_settings.show_physics_visualization = value

    def set_gravity(self, gravity=9.81):
        if isinstance(gravity, float):
            bpy.data.scenes[0].game_settings.physics_gravity = gravity

    def set_viewport(self, viewport_shade='WIREFRAME'):
        """ set_viewport
        :param viewport_shade: enum in ['BOUNDBOX', 'WIREFRAME', 'SOLID', 'TEXTURED'], default 'WIREFRAME'
        """
        for area in bpy.context.window.screen.areas:
            if area.type == 'VIEW_3D':
                for space in area.spaces:
                    if space.type == 'VIEW_3D':
                        space.viewport_shade = viewport_shade

    def set_auto_start(self, auto_start=True):
        bpy.context.scene.render.engine = 'BLENDER_GAME'
        bpy.context.scene.game_settings.use_auto_start = auto_start

    def set_debug(self, debug=True):
        bpy.app.debug = debug

    def set_stereo(self, stereo='STEREO'):
        """ set_stereo
        :param stereo: enum in ['NONE', 'STEREO', 'DOME'], default 'STEREO'
        """
        bpy.context.scene.game_settings.stereo = stereo

    def configure_multinode(self, protocol='socket', 
            server_address='localhost', server_port='65000', distribution=None):
        self._protocol = protocol
        self._server_address = server_address
        self._server_port = server_port
        if distribution != None:
            self.multinode_distribution = distribution
        self._multinode_configured = True

    def configure_service(self, mw):
        AbstractComponent.configure_service(self, mw, "simulation")

    def select_display_camera(self, robot_camera):
        """ Select the camera that will be displayed on the Screen object
        :param robot_camera: AbstractComponent reference to the camera desired to be displayed
        """
        self._display_camera = robot_camera

    def _set_scren_mat(self):
        """ Set the material of the Screen object to the same as the one indicated in the _display_camera variable
        """
        camera = None
        screen = bpy.data.objects['Screen']
        caption = bpy.data.objects['CameraID_text']
        blender_component = self._display_camera._blendobj
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

    def __del__(self):
        """ Call the create method if the user has not explicitly called it """
        if not self._created:
            self.create()
