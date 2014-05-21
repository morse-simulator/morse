import logging; logger = logging.getLogger("morsebuilder." + __name__)
import sys
import math
from morse.builder.blenderobjects import Cube
from morse.builder import bpymorse
from morse.builder.abstractcomponent import AbstractComponent
from morse.core.exceptions import *

"""
Morse Builder API

To test this module you can c/p the following code in Blender Python console::

.. code-block:: python

    import sys
    sys.path.append("/usr/local/lib/python3/dist-packages")
    from morse.builder import *
    atrv=ATRV()

The string passed to the differents Components Classes must be an existing
.blend file-name, ie. for ``ATRV()`` the file ``atrv.blend`` must exists
in the folder ``MORSE_COMPONENTS/robots/``.
"""

# Override the default Python exception handler
sys_excepthook = sys.excepthook
def morse_excepthook(*args, **kwargs):
    logger.error("[ERROR][MORSE] Uncaught exception, quit Blender.", exc_info = tuple(args))
    sys_excepthook(*args, **kwargs)
    import os
    os._exit(-1)

# Uncaught exception quit Blender
sys.excepthook = morse_excepthook

class PassiveObject(AbstractComponent):
    """ Allows to import any Blender object to the scene.
    """

    def __init__(self, filename="props/objects", prefix=None, keep_pose=False):
        """ Initialize a PassiveObject

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

        logger.info("Importing the following passive object(s): %s" % prefix)

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

class Zone(Cube):
    def __init__(self, type):
        Cube.__init__(self, 'xxx')
        # Need to create a new material before calling make_transparent
        self._bpy_object.active_material = bpymorse.create_new_material()
        self._make_transparent(self._bpy_object, 1e-6)
        self.properties(Zone_Tag = True, Type = type)

    @property
    def size(self):
        return self._bpy_object.scale
    @size.setter
    def size(self, value):
        self._bpy_object.scale = value

    def rotate(self, x=0.0, y=0.0, z=0.0):
        logger.warning("rotate is not supported for Zone")

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


class Robot(Component):
    def __init__(self, filename = '', name = None):
        Component.__init__(self, 'robots', filename)
        self.properties(Robot_Tag = True)
        self.default_interface = None
        if name:
            self.name = name

    def set_friction(self, friction=0.0):
        """ Set  Coulomb friction coefficient

        :param friction: [0, 100], default 0.0
        :type  friction: float
        """
        for slot in self._bpy_object.material_slots: # ['TireMat']
            slot.material.physics.friction = friction

    def set_mass(self, mass):
        """ Set component's mass

        :param mass: The component's mass
        :type  mass: float

        .. note::
            The object must have a physics controller for the mass to be
            applied, otherwise the mass value will be returned as 0.0.
        """
        self._bpy_object.game.mass = mass

    def add_default_interface(self, stream):
        """ Add a service and stream interface to all components of the robot

        .. note::
            If add_stream or add_service is used explicitly for some
            components and the specified interface is the same it will be
            added twice.
        """
        self.default_interface = stream

    def make_external(self):
        self._bpy_object.game.properties['Robot_Tag'].name = 'External_Robot_Tag'
    
    def make_ghost(self, alpha=0.3):
        """ Make this robot a ghost
        
        The robot is made transparent, with no collision.
        
        .. note::
             A ghost robot has no influence on other simulated robots
             (no collision, invisible to laser sensors) except for video sensors.
        
        :param alpha: Transparency alpha coefficient (0 for invisible, 1 for opaque, default is 0.3)
        """
        self._make_transparent(self._bpy_object, alpha)

    def set_rigid_body(self):
        """ Configure this robot to use rigid_body physics """
        self._bpy_object.game.use_actor = True
        self._bpy_object.game.physics_type = 'RIGID_BODY'
        self._bpy_object.game.use_sleep = True

    def set_no_collision(self):
        """ Configure this robot to not use physics at all """
        self._bpy_object.game.physics_type = 'NO_COLLISION'

    def set_dynamic(self):
        self._bpy_object.game.physics_type = 'DYNAMIC'
        self._bpy_object.game.use_actor = True
        self._bpy_object.game.use_sleep = True

    def set_collision_bounds(self):
        self._bpy_object.game.use_collision_bounds = True
        self._bpy_object.game.collision_bounds_type = 'CONVEX_HULL'
        self._bpy_object.game.use_collision_compound = True

class GroundRobot(Robot):
    def __init__(self, filename, name):
        Robot.__init__(self, filename, name)
        self.properties(GroundRobot = True)

class WheeledRobot(GroundRobot):
    def __init__(self, filename, name):
        Robot.__init__(self, filename, name)

    def unparent_wheels(self):
        """ Make the wheels orphans, but keep the transformation applied to
            the parent robot """
        # Force Blender to update the transformation matrices of objects
        bpymorse.get_context_scene().update()

        wheels = [child for child in self._bpy_object.children if \
                  "wheel" in child.name.lower()]
        wnames = ['None'] * 5
        keys = ['WheelFLName', 'WheelFRName', 'WheelRLName',
                'WheelRRName', 'CasterWheelName']
        properties = bpymorse.get_properties(self._bpy_object)
        for i in range(5):
            key = keys[i]
            expected_wheel = properties.get(key, None)
            if expected_wheel:
                for wheel in wheels:
                    if wheel.name.startswith(expected_wheel):
                        wnames[i] = wheel.name

        self.properties(WheelFLName = wnames[0], WheelFRName = wnames[1],
                        WheelRLName = wnames[2], WheelRRName = wnames[3],
                        CasterWheelName = wnames[4])
        for wheel in wheels:
            # Make a copy of the current transformation matrix
            transformation = wheel.matrix_world.copy()
            wheel.parent = None
            wheel.matrix_world = transformation
            # This method should be easier, but does not seem to work
            #  because of an incorrect context error
            #bpy.ops.object.parent_clear(type='CLEAR_KEEP_TRANSFORM')

    def after_renaming(self):
        self.unparent_wheels()
        for obj in self.children:
            old = obj._bpy_object.rotation_euler
            obj._bpy_object.rotation_euler = (old[0], old[1], old[2]+math.pi/2)

            # Switch the values of X and Y location
            tmp_x = obj._bpy_object.location[0]
            obj._bpy_object.location[0] = -obj._bpy_object.location[1]
            obj._bpy_object.location[1] = tmp_x



class Sensor(Component):
    def __init__(self, filename):
        Component.__init__(self, 'sensors', filename)
        self.properties(Component_Tag = True)

class Actuator(Component):
    def __init__(self, filename = None):
        Component.__init__(self, 'actuators', filename)
        self.properties(Component_Tag = True)


class Armature(Actuator):
    def __init__(self, armature_name = None, model_name = None):
        """ Initialize an Armature

        :param armature_name: Armature object name
        :param model_name: Armature model name, if any
        """
        Actuator.__init__(self, filename = model_name)

        if not armature_name and not model_name:
            raise MorseBuilderError("You need to specify either the name of " \
                    "an armature or a Blender model in order to create an " \
                    "armature actuator.")

        if armature_name:
            if not model_name:
                # no model name given -> MORSE automatically added an empty
                # for this actuator, which we do not want since we provide
                # our own Blender armature.
                bpymorse.delete([self._bpy_object])
            self.set_blender_object(bpymorse.get_object(armature_name))
            # explictly recall morseable(), as it impacts on
            # blender_objer, and we just change it.
            self.morseable()
            self.properties(Component_Tag = True)

        # default classpath for Armature (can be modified)
        self.properties(classpath="morse.actuators.armature.Armature")

        self.ik_targets = []

        # the user may have created IK constraints on the armature, without
        # setting an IK target. In that case, we add such a target
        for bone in self._bpy_object.pose.bones:
            for c in bone.constraints:
                if c.type == 'IK' and c.ik_type == 'DISTANCE':
                    if not c.target:
                        self.create_ik_targets([bone.name])

    def _get_posebone(self, bone_name):
        """ Returns a given PoseBone in the armature.

        If the joint does not exist, throw an exception.
        """
        armature = self._bpy_object

        if bone_name not in [c.name for c in armature.pose.bones]:
            msg = "Joint <%s> does not exist in model %s." % (bone_name, armature.name)
            msg += " Did you add a skeleton to your model in MakeHuman?"
            raise MorseBuilderError(msg)

        return armature.pose.bones[bone_name]

    def create_ik_targets(self, bones):

        # Bug with iTaSC! cf http://developer.blender.org/T37894
        if bpymorse.version() < (2, 70, 0):
            if self._bpy_object.pose.ik_solver == 'ITASC':
                logger.warn("Due to a bug in Blender (T37894), only the standard " \
                            "IK solver can be used with IK targets. Switching " \
                            "from iTaSC to standard IK solver.")
                self._bpy_object.pose.ik_solver = 'LEGACY'

        for target in bones:
            posebone = self._get_posebone(target)
            bpymorse.add_morse_empty("ARROWS")
            empty = bpymorse.get_first_selected_object()
            empty.scale = [0.01, 0.01, 0.01]

            empty.matrix_local = posebone.bone.matrix_local
            empty.location = posebone.bone.tail_local

            existing_ik = [c for c in posebone.constraints if c.type == 'IK']
            if len(existing_ik) == 1:
                ik_constraint = existing_ik[0]
            elif existing_ik:
                raise MorseBuilderError("Bone %s has several IK constraints." \
                        "MORSE supports only one IK constraint per bone. Please " \
                        "remove other ones.")
            else:
                ik_constraint = posebone.constraints.new("IK")

            ik_constraint.ik_type = "DISTANCE"
            ik_constraint.use_rotation = True
            ik_constraint.use_tail = True
            ik_constraint.target = empty

            self.ik_targets.append((empty, target))


    def after_renaming(self):
        for empty, target in self.ik_targets:
            empty.name = "ik_target." + self.name + "." + target



