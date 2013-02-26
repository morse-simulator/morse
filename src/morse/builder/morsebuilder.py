import logging; logger = logging.getLogger("morsebuilder." + __name__)
import math
from morse.builder.abstractcomponent import *

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
        AbstractComponent.__init__(self, filename=filename, category='actuators')
        self.set_blender_object(bpymorse.get_object(objectname))
        # default classpath for Armature (can be modified)
        if not self.is_morseable():
            self.morseable()
        self.properties(classpath="morse.actuators.armature.Armature")


class Robot(Component):
    def __init__(self, filename):
        Component.__init__(self, 'robots', filename)
        self.properties(Robot_Tag = True)

    def add_default_interface(self, stream):
        for child in self.children:
            if child.is_morseable():
                child.add_interface(stream)

    def make_external(self):
        self._bpy_object.game.properties['Robot_Tag'].name = 'External_Robot_Tag'

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
        """ Add a child to the current object

        Overloads :py:meth:`morse.builder.abstractcomponent.AbstractComponent.append`
        *e.g.*, ``robot.append(sensor)`` will set the robot parent of the sensor.
        """
        # Correct the rotation of the object
        old = obj._bpy_object.rotation_euler
        obj._bpy_object.rotation_euler = (old[0], old[1], old[2]+math.pi/2)

        # Switch the values of X and Y location
        tmp_x = obj._bpy_object.location[0]
        obj._bpy_object.location[0] = -obj._bpy_object.location[1]
        obj._bpy_object.location[1] = tmp_x

        Robot.append(self, obj, 2)

    def after_renaming(self):
        self.unparent_wheels()



class Sensor(Component):
    def __init__(self, filename):
        Component.__init__(self, 'sensors', filename)
        self.properties(Component_Tag = True)

class Actuator(Component):
    def __init__(self, filename):
        Component.__init__(self, 'actuators', filename)
        self.properties(Component_Tag = True)
