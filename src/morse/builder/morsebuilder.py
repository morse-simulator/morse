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
def morse_excepthook(*args, **kwargs):
    logger.error("[ERROR][MORSE] Uncaught exception, quit Blender.", exc_info = tuple(args))
    # call default python exception hook
    # on Ubuntu/Python3.4 sys.excepthook is overriden by `apport_excepthook`
    sys.__excepthook__(*args, **kwargs)
    import os
    os._exit(-1)

# Uncaught exception quit Blender
sys.excepthook = morse_excepthook
# workaround avoid numpy.core.multiarray missmatch ( see #630 )
sys.path.insert(0, '%s/lib/python%i.%i/site-packages'%(sys.prefix,
    sys.version_info.major, sys.version_info.minor))

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
    def __init__(self, category='', filename='',blender_object_name=None, make_morseable=True):
        """ Initialize a MORSE component

        :param category: The category of the component (folder in
            MORSE_COMPONENTS)
        :param filename: The name of the component (file in
            MORSE_COMPONENTS/category/name.{blend, urdf}).
            If it ends with '.blend', append the objects from the Blender 
            file. If it ends with '.urdf', it attemps to laod the URDF file.
        :param blender_object_name: If set, use the given Blender object 
            as 'root' for this component. Otherwise, select the first
            available Blender object (the top parent in case of a hierarchy
            of objects).
        :param make_morseable: If the component has no property for the
            simulation, append default Morse ones. See self.morseable()
        """
        AbstractComponent.__init__(self, filename=filename, category=category)

        if self.has_urdf:
            root = self.load_urdf()
            self.set_blender_object(root)

        else:
            imported_objects = self.append_meshes()

            if blender_object_name is None:
                # Here we use the fact that after appending, Blender select the objects
                # and the root (parent) object first ( [0] )
                self.set_blender_object(imported_objects[0])
            else:
                self.set_blender_object([o for o in imported_objects if o.name == blender_object_name][0])

        # If the object has no MORSE logic, add default one
        if make_morseable and category in ['sensors', 'actuators', 'robots'] \
                and not self.is_morseable():
            self.morseable()


class Robot(Component):
    def __init__(self, filename='', name=None, blender_object_name=None):
        """ Initialize a MORSE robot

        :param filename: The name of the component (file in
            MORSE_COMPONENTS/category/name.{blend, urdf}).
            If it ends with '.blend', append the objects from the Blender 
            file. If it ends with '.urdf', it attemps to laod the URDF file.
        :param name: Name of the resulting robot in the simulation, default to 'robot'.
        :param blender_object_name: If set, use the given Blender object 
            in 'filename' as 'root' for this robot. Otherwise, select the first
            available Blender object (the top parent in case of a hierarchy
            of objects).
        """
        Component.__init__(self, 'robots', filename, blender_object_name=blender_object_name)
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

    def set_physics_type(self, physics_type='STATIC'):
        """ Configure this robot physics type """
        self._bpy_object.game.physics_type = physics_type

    def set_use_record_animation(self, use_record_animation=True):
        """ Record animation objects without physics """
        self._bpy_object.game.use_record_animation = use_record_animation

    def set_dynamic(self):
        self._bpy_object.game.physics_type = 'DYNAMIC'
        self._bpy_object.game.use_actor = True
        self._bpy_object.game.use_sleep = True

    def set_collision_bounds(self):
        self._bpy_object.game.use_collision_bounds = True
        self._bpy_object.game.collision_bounds_type = 'CONVEX_HULL'
        self._bpy_object.game.use_collision_compound = True

    def make_grasper(self, obj_name):
        obj = bpymorse.get_object(obj_name)
        bpymorse.select_only(obj)
        bpymorse.add_sensor(type = 'NEAR')
        sens = obj.game.sensors[-1]
        sens.name = 'Near'
        sens.distance = 5.0
        sens.reset_distance = 0.075
        sens.property = "Graspable"
        bpymorse.add_controller()
        contr = obj.game.controllers[-1]
        contr.link(sensor = sens)


class GroundRobot(Robot):
    def __init__(self, filename, name, blender_object_name=None):
        Robot.__init__(self, filename, name, blender_object_name=blender_object_name)
        self.properties(GroundRobot = True)

class WheeledRobot(GroundRobot):
    def __init__(self, filename, name, blender_object_name=None):
        Robot.__init__(self, filename, name, blender_object_name=blender_object_name)

    def unparent_wheels(self):
        """ Make the wheels orphans, but keep the transformation applied to
            the parent robot """
        # Force Blender to update the transformation matrices of objects
        bpymorse.get_context_scene().update()

        keys = ['WheelFLName', 'WheelFRName', 'WheelRLName',
                'WheelRRName', 'CasterWheelName']
        properties = bpymorse.get_properties(self._bpy_object)
        for key in keys:
            expected_wheel = properties.get(key, None)
            if expected_wheel:
                wheel = self.get_child(expected_wheel)
                if wheel:
                    # Make a copy of the current transformation matrix
                    transformation = wheel.matrix_world.copy()
                    wheel.parent = None
                    wheel.matrix_world = transformation
                else:
                    logger.error('Wheel %s is required but not found' % expected_wheel)

    def after_renaming(self):
        self.unparent_wheels()
