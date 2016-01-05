import logging; logger = logging.getLogger("morse." + __name__)
import math
from morse.core.blenderapi import mathutils

from collections import OrderedDict
import morse.core.actuator
from morse.core import status
from morse.core import blenderapi
from morse.core.services import service, async_service, interruptible
from morse.core.exceptions import MorseRPCInvokationError
from morse.core.morse_time import time_isafter
from morse.helpers.morse_math import normalise_angle
from morse.helpers.components import add_property

class Armature(morse.core.actuator.Actuator):
    """
    **Armatures** are the MORSE generic way to simulate kinematic chains
    made of a combination of revolute joints (hinge) and prismatic
    joints (slider).

    This component only allows to *set* an armature configuration. To read the
    armature pose, you need an :doc:`armature pose sensor <../sensors/armature_pose>`.

    .. important:: 

        To be valid, special care must be given when creating armatures. If you
        want to add new one, please carefully read the :doc:`armature creation
        <../../dev/armature_creation>` documentation.


    This actuator offers two main ways to control a kinematic chain: either by
    setting the values of each joint individually (via a continuous datastream
    or via dedicated services: :py:meth:`translate`,
    :py:meth:`set_translation`, :py:meth:`rotate`, :py:meth:`set_rotation`) or
    by placing the end-effector and relying on a inverse kinematic solver (via
    the services :py:meth:`set_IK_target` and :py:meth:`move_IK_target`).

    .. note::

        When setting the joints with a datastream, the data structure that the
        armature actuator expects depends on the armature itself.  It is a
        dictionary of pair `(joint name, joint value)`.  Joint values are
        either radians (for revolute joints) or meters (for prismatic joints)

    To use inverse kinematics, you must first define *IK targets* to control
    your kinematic chain. You can create them :doc:`manually from Blender
    <../../dev/armature_creation>` or directly in your `Builder script` (refer
    to the `Examples` section below for an example).

    .. example::

        from morse.builder import *

        robot = ATRV()

        # imports an armature,
        # either from a Blender file...
        armature = Armature(model_name = "<blend file>")
        # ...or by providing the name of an existing armature:
        # armature = Armature(armature_name = "<name of armature>")
        # (note that you can combine both options to select an armature in a 
        # Blender file)

        # if you want to use inverse kinematics, you can create 'IK targets'
        # here as well:
        armature.create_ik_targets(["<name of the bone you want to control>", ...])

        # place your armature at the correct location
        armature.translate(<x>, <y>, <z>)
        armature.rotate(<rx>, <ry>, <rz>)

        # define one or several communication interfaces, like 'socket' or 'ros'
        # With ROS, this actuator exposes a JointTrajectoryAction interface.
        armature.add_interface(<interface>)

        robot.append(armature)

        env = Environment('empty')
        
    .. note::

        :tag:`ros` Armatures can be controlled in ROS through the
        `JointTrajectoryAction
        <http://wiki.ros.org/joint_trajectory_action>`_
        interface.

    :sees: :doc:`armature pose sensor <../sensors/armature_pose>`

    :noautoexample:
    """
    _name = "Armature Actuator"
    _short_desc="An actuator to manipulate Blender armatures in MORSE."

    add_property('distance_tolerance', 0.005, 'DistanceTolerance', 'float', "Tolerance in meters when translating a joint")
    add_property('angle_tolerance', 0.01, 'AngleTolerance', 'float', "Tolerance in radians when rotating a joint")
    add_property('radial_speed', 0.8, 'RotationSpeed', 'float', "Global rotation speed for the armature rotational joints (in rad/s)")
    add_property('linear_speed', 0.05, 'LinearSpeed', 'float', "Global linear speed for the armature prismatic joints (in m/s)")
    add_property('ik_target_radial_speed', 0.5, 'IKRotationSpeed', 'float', "Default speed of IK target rotation (in rad/s)")
    add_property('ik_target_linear_speed', 0.5, 'IKLinearSpeed', 'float', "Default speed of IK target motion (in m/s)")

    def __init__(self, obj, parent=None):
        """
        Creates a new instance of Armature.

        :param obj: the Blender **armature** object that is to be controlled.
        """     
        # Call the constructor of the parent class
        morse.core.actuator.Actuator.__init__(self, obj, parent)
        
        # Initialize the values in local_data for each segment
        armature = self.bge_object

        for channel in armature.channels:
            self.local_data[channel.name] = 0.0

        self._ik_targets = {c.target: c for c in armature.constraints \
                            if c.type == blenderapi.CONSTRAINT_TYPE_KINEMATIC and \
                               c.ik_type == blenderapi.CONSTRAINT_IK_DISTANCE}

        # Initially desactivate all IK constraints
        for c in self._ik_targets.values():
            c.active = False

        # holds the destinations for the IK targets when async service
        # `move_IK_target` is called.
        self._ik_targets_destinations = {}

        # The axis along which the different segments rotate
        # Considering the constraints defined for the armature
        #  in the Blender file
        self._dofs = self.get_dofs()

        self.joint_speed = {}

        # Look for the armature's end effector, if any
        self._end_effector = None
        for child in self.bge_object.childrenRecursive:
            if 'end_effector' in child:
                logger.info("Found end effector (%s) for armature %s" % (child.name, obj.name))
                self._end_effector= child
                break

        # If we find an end effector, all armature children that do not have
        # the property 'internal' are considered to be mounted on the end
        # effector
        if self._end_effector:
            for child in self.bge_object.children:
                if not 'internal' in child:
                    child.setParent(self._end_effector)

        ####
        # Trajectory specific variables
        self._active_trajectory = None


        logger.info('%s armature initialized with joints [%s].' % (obj.name, ", ".join(self.local_data.keys())))

        if self._ik_targets:
            logger.info("%d IK targets available on this armature: %s." % (len(self._ik_targets), ", ".join([t.name for t in self._ik_targets.keys()])))

    def _suspend_ik_targets(self):
        for c in self._ik_targets.values():
            #Bug in Blender! cf http://developer.blender.org/T37892
            if blenderapi.version() < (2, 70, 0):
                if not c.active:
                    logger.info("Stop tracking IK target <%s>" % c.target.name)
                    c.active = False
            else:
                if c.active:
                    logger.info("Stop tracking IK target <%s>" % c.target.name)
                    c.active = False


    def _restore_ik_targets(self):
        for c in self._ik_targets.values():
            #Bug in Blender! cf http://developer.blender.org/T37892
            if blenderapi.version() < (2, 70, 0):
                if c.active:
                    c.active = True
                    logger.info("Tracking IK target <%s>" % c.target.name)
            else:
                if not c.active:
                    c.active = True
                    logger.info("Tracking IK target <%s>" % c.target.name)


    def _is_prismatic(self, channel):
        """
        Important: The detection of prismatic joint relies solely on a
        non-zero value for the IK parameter 'ik_stretch'.
        """
        return True if channel.ik_stretch else False

    def _get_joint(self, joint):
        """ Checks a given joint name exist in the armature,
        and returns it as a tuple (Blender channel, is_prismatic?)

        If the joint does not exist, throw an exception.
        """
        armature = self.bge_object

        if joint not in [c.name for c in armature.channels]:
            msg = "Joint <%s> does not exist in armature %s" % (joint, armature.name)
            raise MorseRPCInvokationError(msg)

        channel = armature.channels[joint]

        if self._is_prismatic(channel):
            return channel, True
        else:
            return channel, False

    def _get_joint_value(self, joint):
        """
        Returns the *value* of a given joint, either:
        - its absolute rotation in radian along its rotation axis, or
        - it absolute translation in meters along its translation axis.

        Throws an exception if the joint does not exist.

        :param joint: the name of the joint in the armature.
        """
        channel, is_prismatic = self._get_joint(joint)

        # Retrieve the motion axis
        axis_index = next(i for i, j in enumerate(self.find_dof(channel)) if j)

        if is_prismatic:
            return channel.pose_head[2] #The 'Z' value
        else: # revolute joint
            return channel.joint_rotation[axis_index]

    def _get_prismatic(self, joint):
        """ Checks a given prismatic joint name exist in the armature, and
        returns it.
        """
        channel, is_prismatic = self._get_joint(joint)

        if not is_prismatic:
            msg = "Joint %s is not a prismatic joint! " \
                  "Can not set the translation" % joint
            raise MorseRPCInvokationError(msg)

        return channel

    def _get_revolute(self, joint):
        """ Checks a given revolute joint name exist in the armature, and
        returns it.
        """
        channel, is_prismatic = self._get_joint(joint)

        if is_prismatic:
            msg = "Joint %s is not a revolute joint! Can not set the rotation" % joint
            raise MorseRPCInvokationError(msg)

        return channel

    def _clamp_joint(self, channel, rotation):

        ik_min, ik_max = self.get_IK_limits(channel.name)
        return max(ik_min, min(rotation, ik_max))

    @service
    def list_IK_targets(self):
        return [ik.name for ik in self._ik_targets.keys()]

    def _normalize_IK_transformation(self, 
                                     target_name, 
                                     translation, 
                                     euler_rotation = None, 
                                     relative = True):

        armature = self.bge_object
        target = [ik for ik in self._ik_targets.keys() if ik.name == target_name]
        if not target:
            raise MorseRPCInvokationError("IK target <%s> does not exist for armature %s" % (target_name, armature.name))

        target = target[0]

        if relative:
            currentPos = target.worldPosition
            translation = [translation[0] + currentPos[0],
                           translation[1] + currentPos[1],
                           translation[2] + currentPos[2]]

        if euler_rotation:
            current_orientation = target.worldOrientation.to_quaternion().normalized()
            rotation = mathutils.Euler(euler_rotation).to_quaternion().normalized()

            if relative:
                # computes the final IK target orientation by rotating the
                # current orientation by `rotation`
                final_orientation = current_orientation.copy()
                final_orientation.rotate(rotation)
            else:
                final_orientation = rotation
        else:
            final_orientation = None

        return target, translation, final_orientation

    @service
    def place_IK_target(self, name, translation, euler_rotation = None, relative = True):
        """
        Places instantaneously a IK (inverse kinematic) target to a given
        position and orientation.

        :sees: `move_IK_target` to move the IK target over time.

        :param name: name of the IK target (as returned by
        :py:meth:`list_IK_targets`)
        :param translation: a [x,y,z] translation vector, in the scene frame,
        in meters.
        :param rotation: a [rx,ry,rz] rotation, in the scene frame (ie, X,Y,Z 
        rotation axis are the scene axis). Angles in radians.
        :param relative: if True (default), translation and rotation are 
        relative to the current target pose.

        """
        self._restore_ik_targets()

        target, translation, rotation = self._normalize_IK_transformation(
                                                        name, 
                                                        translation, 
                                                        euler_rotation, 
                                                        relative)

        target.worldPosition = translation
        if rotation:
            target.worldOrientation = rotation

        # save the joint state computed from IK in local_data
        self._store_current_joint_state()

    @interruptible
    @async_service
    def move_IK_target(self, name, 
                             translation, euler_rotation = None, 
                             relative = True, 
                             linear_speed = None, radial_speed = None):
        """
        Moves an IK (inverse kinematic) target at a given speed (in m/s for
        translation, rad/s for rotation).

        Note that moving an IK target conflicts somewhat with the original
        purpose of the inverse kinematic solver, and overall continuity is not
        guaranteed (the IK solver may find a solution for a given target
        position that 'jumps' relative to the solution for the previous target
        position).

        :sees: `place_IK_target` to set instantaneously the IK target pose.

        :param name: name of the IK target (as returned by :py:meth:`list_IK_targets`)
        :param translation: a [x,y,z] translation vector, in the scene frame, 
        in meters.
        :param rotation: a [rx,ry,rz] rotation, in the scene frame (ie, X,Y,Z 
        rotation axis are the scene axis). Angles in radians.
        :param relative: if True (default), translation and rotation are 
        relative to the current target pose.
        :param linear_speed: (default: value of the `ik_target_linear_speed` 
        property) translation speed (in m/s).
        :param radial_speed: (default: value of the `ik_target_radial_speed` 
        property) rotation speed (in rad/s).

        """

        self._restore_ik_targets()

        target, translation, final_orientation = self._normalize_IK_transformation(
                                                            name, 
                                                            translation, 
                                                            euler_rotation, 
                                                            relative)

        if not linear_speed:
            linear_speed = self.ik_target_linear_speed
        if not radial_speed:
            radial_speed = self.ik_target_radial_speed

        if final_orientation:
            current_orientation = target.worldOrientation.to_quaternion().normalized()

            # uses mathutils.Vector.angle to return the angle in radians
            # between the 2 orientations
            radian_distance = current_orientation.axis.angle(final_orientation.axis)

            # we need to compute at initialization the total expected duration
            # of the rotation since quaternion interpolation relies on
            # Quaternion.slerp that takes a interpolation factor between 0.0
            # and 1.0.  During the rotation execution, we compute the factor
            # based on the current rotation duration and the total expected
            # duration
            initial_time_rotation = self.robot_parent.gettime() # in seconds
            total_rotation_duration = (radian_distance / radial_speed) 
        else:
            current_orientation = final_orientation = None
            initial_time_rotation = total_rotation_duration = None

        self._ik_targets_destinations[target] = (translation, 
                                                 linear_speed, 
                                                 current_orientation, 
                                                 final_orientation, 
                                                 initial_time_rotation, 
                                                 total_rotation_duration)

    @service
    def set_translation(self, joint, translation):
        """
        Translates instantaneously the given (prismatic) joint by the given
        translation. Joint speed limit is not taken into account.

        :sees: `Blender documentation on joint location <http://www.blender.org/documentation/blender_python_api_2_64_release/bge.types.html#bge.types.BL_ArmatureChannel.location>`_

        If the joint does not exist or is not a prismatic joint (slider),
        throws a MorseServiceFailed exception.

        The translation is always clamped to the joint limit.

        :param joint: name of the joint to move
        :param translation: absolute translation from the joint origin in the
        joint sliding axis, in meters

        """

        self._suspend_ik_targets()

        channel = self._get_prismatic(joint)

        # Retrieve the translation axis
        axis_index = next(i for i, j in enumerate(self.find_dof(channel)) if j)

        translation = self._clamp_joint(channel, translation)

        self.local_data[channel.name] = translation

        tmp = channel.location
        tmp[axis_index] = translation
        channel.location = tmp

    @service
    def set_translations(self, translations):
        """
        Sets in one call the translations of the prismatic joints in this
        armature.

        Has the same effect as applying `set_translation` on each of the joints
        independantly.

        Translations must be ordered from the root to the tip of the armature.
        Use the service ``get_joints()`` to retrieve the list of joints
        in the correct order.

        If more translations are provided than the number of joints, the
        remaining ones are discarded. If less translations are provided, the
        maximum are applied.

        .. important::

            If a revolute joint is encountered while applying the translations,
            an exception is thrown, and **no** translation is applied.

        :sees: `set_translation`
        :param translations: a set of absolute translations, in meters
        """
        armature = self.bge_object

        nb_trans = min(len(translations), len(armature.channels))

        channels = [c for c in armature.channels]
        for i in range(nb_trans):
            if not self._is_prismatic(channels[i]):
                msg = "Joint %s is not a prismatic joint! " \
                      "Can not apply the translation set" % channels[i].name
                raise MorseRPCInvokationError(msg)

        for trans, channel in zip(translations[:nb_trans], channels[:nb_trans]):
            self.set_translation(channel.name, trans)


    @interruptible
    @async_service
    def translate(self, joint, translation, speed = None):
        """
        Translates a joint at a given speed (in m/s).

        :param joint: name of the armature's joint to translate
        :param translation: the absolute translation, relative to the joint
        origin, in meters
        :param speed: (default: value of 'linear_speed' property) translation
        speed, in m/s

        """

        self._suspend_ik_targets()

        channel = self._get_prismatic(joint) # checks the joint exist and is prismatic
        translation = self._clamp_joint(channel, translation)
        self.joint_speed[joint] = speed

        logger.info("Initiating translation of joint %s to %s"%(joint, translation))
        self.local_data[joint] = translation

    @interruptible
    @async_service
    def translate_joints(self, joint_translations, speed = None):
        """
        Translates a set of joints at a given speed (in m/s).

        .. note::

            Setting different speeds for each joints is not yet supported (but
            if you need it, ping morse-dev@laas.fr: it is fairly easy to add).

        :param joint_translations: a list of mapping {name of the armature's joint: translation in meters}
        :param speed: (default: value of 'linear_speed' property) rotation speed for all joints, in m/s
        """

        self._suspend_ik_targets()

        for joint, translation in joint_translations.items():
            channel = self._get_prismatic(joint) # checks the joint exist and is prismatic
            translation = self._clamp_joint(channel, translation)
            self.joint_speed[joint] = speed

            logger.info("Initiating translation of joint %s to %s"%(joint, translation))
            self.local_data[joint] = translation


    @service
    def set_rotation(self, joint, rotation):
        """
        Rotates instantaneously the given (revolute) joint by the given
        rotation. Joint speed limit is not taken into account.

        If the joint does not exist or is not a revolute joint (hinge),
        throws a MorseServiceFailed exception.

        The rotation is always clamped to the joint limit.

        :param joint: name of the joint to rotate
        :param rotation: absolute rotation from the joint origin along the
        joint rotation axis, in radians

        """

        self._suspend_ik_targets()

        channel = self._get_revolute(joint)

        # Retrieve the translation axis
        axis_index = next(i for i, j in enumerate(self.find_dof(channel)) if j)

        rotation = self._clamp_joint(channel, rotation)

        self.local_data[channel.name] = rotation

        tmp = channel.joint_rotation
        tmp[axis_index] = rotation
        channel.joint_rotation = tmp

    @service
    def set_rotations(self, rotations):
        """
        Sets in one call the rotations of the revolute joints in this armature.

        Has the same effect as applying `set_rotation` on each of the joints
        independantly.

        Rotations must be ordered from the root to the tip of the armature. Use
        the service ``get_joints()`` to retrieve the list of joints in the
        correct order.

        If more rotations are provided than the number of joints, the remaining
        ones are discarded. If less rotations are provided, the maximum are
        applied.

        .. important::

            If a prismatic joint is encountered while applying the rotation,
            an exception is thrown, and **no** rotation is applied.

        :sees: `set_rotation`
        :param rotations: a set of absolute rotations, in radians
        """
        armature = self.bge_object

        nb_rot = min(len(rotations), len(armature.channels))

        channels = [c for c in armature.channels]
        for i in range(nb_rot):
            if self._is_prismatic(channels[i]):
                msg = "Joint %s is not a revolute joint! Can not apply the rotation set" % joint
                raise MorseRPCInvokationError(msg)

        for rot, channel in zip(rotations[:nb_rot], channels[:nb_rot]):
            self.set_rotation(channel.name, rot)



    @interruptible
    @async_service
    def rotate(self, joint, rotation, speed = None):
        """
        Rotates a joint at a given speed (in rad/s).

        :sees: `Blender documentation on joint rotation <http://www.blender.org/documentation/blender_python_api_2_64_release/bge.types.html#bge.types.BL_ArmatureChannel.joint_rotation>`_

        :param joint: name of the armature's joint to rotate
        :param rotation: rotation around the joint axis in radians
        :param speed: (default: value of 'radial_speed' property) rotation speed, in rad/s
        """

        self._suspend_ik_targets()

        channel = self._get_revolute(joint) # checks the joint exist and is revolute
        rotation = self._clamp_joint(channel, rotation)
        self.joint_speed[joint] = speed

        logger.info("Initiating rotation of %s to pos. %f (along the joint rotation axis)"%(joint, rotation))
        self.local_data[joint] = rotation

    @interruptible
    @async_service
    def rotate_joints(self, joint_rotations, speed = None):
        """
        Rotates a set of joints at a given speed (in rad/s).

        :sees: `Blender documentation on joint rotation <http://www.blender.org/documentation/blender_python_api_2_64_release/bge.types.html#bge.types.BL_ArmatureChannel.joint_rotation>`_

        .. note::

            Setting different speeds for each joints is not yet supported (but
            if you need it, ping morse-dev@laas.fr: it is fairly easy to add).

        :param joint_rotations: a list of mapping {name of the armature's joint: rotation in radian}
        :param speed: (default: value of 'radial_speed' property) rotation speed for all joints, in rad/s
        """

        self._suspend_ik_targets()

        for joint, rotation in joint_rotations.items():
            channel = self._get_revolute(joint) # checks the joint exist and is revolute
            rotation = self._clamp_joint(channel, rotation)
            self.joint_speed[joint] = speed

            logger.info("Initiating rotation of %s to pos. %f (along the joint rotation axis)"%(joint, rotation))
            self.local_data[joint] = rotation

    def find_dof(self, channel):
        """
        Method that finds and returns the degree of freedom (dof) corresponding
        to the given channel.
        The dof has to be a blender_ik_setting.
        Returns a list [x,y,z] with the corresponding dofs as a boolean.
        """
        return [channel.ik_dof_x, 
                channel.ik_dof_y,
                channel.ik_dof_z]

    @service
    def get_dofs(self):
        """
        Returns a dictionary with keys the channels
        of the armature and as values the rotation axes of the joint.

        Rotation axes are represented as a list of boolean. For instance,
        ``[True, False, True]`` means that the `x` and `z` axes of the joint are
        free to rotate (or translate, depending on the type of the joint).

        """
        armature = self.bge_object
        dofs = {}
        # find the dof of each channel
        for channel in armature.channels:
            dofs[channel.name] = self.find_dof(channel)

        return dofs

    @service
    def get_joints(self):
        """
        Returns the names of all the joints in this armature, ordered from the
        root to the tip.
        """
        armature = self.bge_object
        return [channel.name for channel in armature.channels]


    @service
    def get_IK_limits(self, joint):
        """
        Returns the IK limits for the given joint.
        
        - For revolute joints, returns a pair `(ik_min,ik_max)`, in radians.
        - For prismatic joint, returns a pair `(0.0, max translation)`, in meters.
        """

        channel, is_prismatic = self._get_joint(joint)

        if is_prismatic:
            return 0.0, channel.ik_stretch
        else:
            # Retrieve the translation axis
            axis_index = next(i for i, j in enumerate(self.find_dof(channel)) if j)
            if axis_index == 0:
                return (channel.ik_min_x, 
                        channel.ik_max_x)
            elif axis_index == 1:
                return (channel.ik_min_y, 
                        channel.ik_max_y)
            elif axis_index == 2:
                return (channel.ik_min_z, 
                        channel.ik_max_z)

            assert False # should not reach this point.

    @async_service
    def trajectory(self, trajectory):
        """
        Executes a joint trajectory to the armature.

        The `trajectory` parameter should have the following structure:

        .. code-block:: python

            trajectory = {
                'starttime': <timestamp in second>,
                'points': [
                    {'positions': [...],
                     'velocities': [...],
                     'accelerations' [...],
                     'time_from_start': <seconds>}
                    {...},
                    ...
                    ]
                }

        .. warning::

            Currently, both `velocities` and `accelerations` are ignored.

        The trajectory execution starts after `starttime` timestamp passed (if
        omitted, the trajectory execution starts right away).

        `points` is the list of trajectory waypoints. It is assumed that the
        values in the waypoints are ordered the same way as in the set of joint
        of the armature (ie, from the root to the tip of the armature. Use the
        service ``get_joints()`` to retrieve the list of joints in the correct
        order.) `velocities` and `accelerations` are optional.

        The component attempts to achieve each waypoint at the time obtained
        by adding that waypoint's `time_from_start` value to `starttime`.

        :param trajectory: the trajectory to execute, as describe above.
        """

        self._suspend_ik_targets()

        # TODO: support velocities and accelerations via cubic/quintic spline
        # interpolation
        starttime = self.robot_parent.gettime()
        if 'starttime' in trajectory:
            trajectory["starttime"] = max(starttime, trajectory["starttime"])
        else:
            trajectory["starttime"] = starttime

        self._active_trajectory = trajectory

    def _exec_traj(self):

        t = self.robot_parent.gettime()
        trajectory = self._active_trajectory

        try:
            if time_isafter(trajectory["starttime"], t):
                return
            
            if time_isafter(t, trajectory["starttime"] + trajectory["points"][-1]["time_from_start"]):
                #trajectory execution is over!
                self._active_trajectory = None
                # TODO: check here the final pose match the last point pose
                self.completed(status.SUCCESS, None)

            for p in trajectory["points"]:
                end = trajectory["starttime"] + p["time_from_start"]
                if "started" in p and time_isafter(end, t):
                    # currently going to this waypoint: fine!
                    break

                elif "started" not in p and time_isafter(end, t):
                    # start the new waypoint
                    allocated_time = end - t
                    assert(allocated_time > 0)

                    target = OrderedDict(zip(self.local_data.keys(),
                                    p["positions"]))

                    for joint in target.keys():
                        # compute the distance based on actual current joint pose
                        dist = target[joint] - self._get_joint_value(joint)
                        self.joint_speed[joint] = dist/allocated_time

                    self.local_data = target

                    p["started"] = True
                    break

                elif "started" not in p and time_isafter(t, end):
                    logger.warning("Skipped a waypoint on armature <%s>. Wrong 'time_from_start'?" % self.name())

                # case: "started" and t > end: do nothing, go to next point
        except KeyError as ke:
            self._active_trajectory = None
            self.completed(status.FAILED, "Error: invalid trajectory: key %s was expected." % ke)

    def _store_current_joint_state(self):
        for joint in self.local_data.keys():
            self.local_data[joint] = self._get_joint_value(joint)

    def interrupt(self):
    
        self._store_current_joint_state()

        for joint in self.local_data.keys():
            if joint in self.joint_speed:
                del self.joint_speed[joint]

        self._active_trajectory = None

        self._ik_targets_destinations = {}

        morse.core.actuator.Actuator.interrupt(self)

    def _exec_ik_move(self, 
                      target, 
                      location, lspeed, 
                      initial_orientation, final_orientation, 
                      initial_time_rotation, total_rotation_duration):

        armature = self.bge_object

        curPos = target.worldPosition
        curOri = target.worldOrientation.to_quaternion()

        posReached = False
        oriReached = False

        # first, translation
        distx = location[0]-curPos[0]
        disty = location[1]-curPos[1]
        distz = location[2]-curPos[2]

        if     abs(distx) < self.distance_tolerance \
           and abs(disty) < self.distance_tolerance \
           and abs(distz) < self.distance_tolerance:
               posReached = True
        else:
            vx = math.copysign(min(lspeed / self.frequency, abs(distx)), distx)
            vy = math.copysign(min(lspeed / self.frequency, abs(disty)), disty)
            vz = math.copysign(min(lspeed / self.frequency, abs(distz)), distz)
            target.worldPosition = [curPos[0] + vx, curPos[1] + vy, curPos[2] + vz]

        # then, orientation (as quaternion!)
        oriReached = True
        if final_orientation:

            rotation_duration = self.robot_parent.gettime() - initial_time_rotation

            if not rotation_duration > total_rotation_duration:
                oriReached = False
                target.worldOrientation = initial_orientation.slerp(
                                                    final_orientation, 
                                                    rotation_duration / total_rotation_duration)
            else:
                # make sure we eventually reach the final position
                target.worldOrientation = final_orientation 

        # Update the armature to reflect the changes we just performed
        armature.update()

        # save the joint state computed from IK in local_data
        self._store_current_joint_state()

        if posReached and oriReached:
            self.completed(status.SUCCESS, None)
            del self._ik_targets_destinations[target]

    def default_action(self):
        """
        Move the armature according to both the value of local_data

        """

        #TODO: 3 type of async service can be started: joint rotate/translate, 
        # traj execution and IK target move.
        # We need to re-organize the code to check none run at the same time and
        # to keep everything well organized/manageable.

        if self._ik_targets_destinations:

            for k,v in list(self._ik_targets_destinations.items()): # make a copy, as we may delete targets if they are reached
                self._exec_ik_move(k,*v)

            # if we move IK targets, we do not want to do anything else.
            return

        if self._active_trajectory:
            self._exec_traj()


        armature = self.bge_object

        #TODO: we should no have to iterate over the whole armature when we do
        # not have to move at all!
        position_reached = True
        for channel in armature.channels:

            # we assume a joint is prismatic (translation joint) if its IK
            # 'stretch' parameter is non-null
            is_prismatic = self._is_prismatic(channel)

            joint = channel.name

            if joint in self.joint_speed and self.joint_speed[joint]:
                speed = self.joint_speed[joint]
            else:
                speed = self.linear_speed if is_prismatic else self.radial_speed


            # Retrieve the rotation or translation axis
            axis_index = next(i for i, j in enumerate(self.find_dof(channel)) if j)

            if is_prismatic:
                # we take the last index ('Z') of the pose of the HEAD of the
                # bone as the absolute translation of the joint. Not 100% sure
                # it is right...
                dist = self.local_data[joint] - channel.pose_head[2] 
            else:
                dist = self.local_data[joint] - channel.joint_rotation[axis_index]

            w = math.copysign(speed / self.frequency, dist)

            if is_prismatic:
                if not abs(dist) < self.distance_tolerance:
                    position_reached = False

                    trans = channel.location
                    trans[axis_index] += w
                    channel.location = trans
            else:
                if not abs(dist) < self.angle_tolerance:
                    position_reached = False

                    rot = channel.joint_rotation
                    rot[axis_index] += w
                    channel.joint_rotation = rot

            # Update the armature to reflect the changes with just performed
            armature.update()


        if position_reached: # True only when all joints match local_data
            if not self._active_trajectory: # _exec_traj() manage completion for trajectories
                self.completed(status.SUCCESS, None)
            if joint in self.joint_speed:
                del self.joint_speed[joint]


