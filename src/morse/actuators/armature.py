import logging; logger = logging.getLogger("morse." + __name__)
import math
import time
from collections import OrderedDict
import morse.core.actuator
from morse.core import status
from morse.core.services import service, async_service, interruptible
from morse.core.exceptions import MorseRPCInvokationError
from morse.helpers.morse_math import normalise_angle
from morse.helpers.components import add_property

class Armature(morse.core.actuator.Actuator):
    """
    **Armatures** are the MORSE generic way to simulate kinematic chains
    made of a combination of revolute joints (hinge) and prismatic
    joints (slider).

    This component only allows to *write* armature configuration. To read the
    armature pose, you need an :doc:`armature pose sensor <../sensors/armature_pose>`.

    .. important:: 

        To be valid, special care must be given when creating armatures. If you
        want to add new one, please carefully read the :doc:`armature creation
        <../../dev/armature_creation>` documentation.


    .. note::

        The data structure on datastream read by the armature actuator
        depends on the armature.  It is a dictionary of pair `(joint name,
        joint value)`.  Joint values are either radians (for revolute joints)
        or meters (for prismatic joints)

    :sees: :doc:`armature pose sensor <../sensors/armature_pose>`

    .. note::

        :tag:`ros` Armatures can be controlled in ROS through the
        `JointTrajectoryAction
        <http://www.ros.org/wiki/pr2_controllers/Tutorials/Moving%20the%20arm%20using%20the%20Joint%20Trajectory%20Action>`_
        interface.

    """
    _name = "Armature Actuator"
    _short_desc="An actuator to manipulate Blender armatures in MORSE."

    add_property('distance_tolerance', 0.005, 'DistanceTolerance', 'float', "Tolerance in meters when translating a joint")
    add_property('angle_tolerance', 0.01, 'AngleTolerance', 'float', "Tolerance in radians when rotating a joint")
    add_property('radial_speed', 0.8, 'RotationSpeed', 'float', "Global rotation speed for the armature rotational joints (in rad/s)")
    add_property('linear_speed', 0.05, 'LinearSpeed', 'float', "Global linear speed for the armature prismatic joints (in m/s)")

    def __init__(self, obj, parent=None):
        """
        Creates a new instance of Armature.

        :param obj: the Blender **armature** object that is to be controlled.
        """     
        # Call the constructor of the parent class
        super(Armature,self).__init__(obj, parent)
        
        # Initialize the values in local_data for each segment
        armature = self.bge_object
        for channel in armature.channels:
            self.local_data[channel.name] = 0.0


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
        # the property 'internal' are considered to be mounted on the 
        # end effector
        if self._end_effector:
            for child in self.bge_object.children:
                if not 'internal' in child:
                    child.setParent(self._end_effector)

        ####
        # Trajectory specific variables
        self._active_trajectory = None


        logger.info('%s armature initialized with joints [%s].' % (obj.name, ", ".join(self.local_data.keys())))

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
            return (channel, True)
        else:
            return (channel, False)

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
        """ Checks a given prismatic joint name exist in the armature,
        and returns it.
        """
        channel, is_prismatic = self._get_joint(joint)

        if not is_prismatic:
            msg = "Joint %s is not a prismatic joint! Can not set the translation" % joint
            raise MorseRPCInvokationError(msg)

        return channel

    def _get_revolute(self, joint):
        """ Checks a given revolute joint name exist in the armature,
        and returns it.
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
    def set_translation(self, joint, translation):
        """
        Translates instantaneously the given (prismatic) joint by the given
        translation. Joint speed limit is not taken into account.

        :sees: http://www.blender.org/documentation/blender_python_api_2_64_release/bge.types.html#bge.types.BL_ArmatureChannel.location

        If the joint does not exist or is not a prismatic joint (slider),
        throws a MorseServiceFailed exception.

        The translation is always clamped to the joint limit.

        :param joint: name of the joint to move
        :param translation: absolute translation from the joint origin in the joint sliding axis, in meters
        """

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
        Sets in one call the translations of the prismatic joints in this armature.

        Has the same effect as applying `set_translation` on each of the joints
        independantly.

        Translations must be ordered from the root to the tip of the armature.

        If more translations are provided than the number of joints, the remaining ones are discarded. If less translations are provided, the maximum are applied.

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
                msg = "Joint %s is not a prismatic joint! Can not apply the translation set" % joint
                raise MorseRPCInvokationError(msg)

        for trans, channel in zip(translations[:nb_trans], channels[:nb_trans]):
            self.set_translation(channel.name, trans)


    @interruptible
    @async_service
    def translate(self, joint, translation, speed = None):
        """
        Translates a joint at a given speed (in m/s).

        :param joint: name of the armature's joint to translate
        :param translation: the absolute translation, relative to the joint origin, in meters
        :param speed: (default: value of 'linear_speed' property) translation speed, in m/s
        """

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
        :param rotation: absolute rotation from the joint origin along the joint rotation axis, in radians
         """
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

        Rotations must be ordered from the root to the tip of the armature.

        If more rotations are provided than the number of joints, the remaining ones are discarded. If less rotations are provided, the maximum are applied.

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

        :sees: http://www.blender.org/documentation/blender_python_api_2_64_release/bge.types.html#bge.types.BL_ArmatureChannel.joint_rotation

        :param joint: name of the armature's joint to rotate
        :param rotation: rotation around the joint axis in radians
        :param speed: (default: value of 'radial_speed' property) rotation speed, in rad/s
        """
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
        of the armature and as values the rotation axis of the joint.
        """
        armature = self.bge_object
        dofs = {}
        # find the dof of each channel
        for channel in armature.channels:
            dofs[channel.name] = self.find_dof(channel)

        return dofs

    @service
    def get_IK_limits(self, joint):
        """
        Returns the IK limits for the given joint.
        
        - For revolute joints, returns a pair `(ik_min,ik_max)`, in radians.
        - For prismatic joint, returns a pair `(0.0, max translation)`, in meters.
        """

        channel, is_prismatic = self._get_joint(joint)

        if is_prismatic:
            return (0.0, channel.ik_stretch)
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

            assert(False) # should not reach this point.

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

        The trajectory execution starts after `starttime` timestamp passed
        (if omitted, the trajectory execution starts right away).
        
        `points` is the list of trajectory waypoints. It is assumed that the
        values in the waypoints are ordered the same way as in the set of
        joint of the armature (ie, from the root to the tip of the armature).
        `velocities` and `accelerations` are optional.

        The component attempts to achieve each waypoint at the time obtained
        by adding that waypoint's `time_from_start` value to `starttime`.

        :param trajectory: the trajectory to execute, as describe above.
        """

        #TODO: support velocities and accelerations via cubic/quintic spline interpolation
        starttime = time.time() # by default, start now
        if 'starttime' in trajectory:
            trajectory["starttime"] = max(starttime, trajectory["starttime"])
        else:
            trajectory["starttime"] = starttime

        self._active_trajectory = trajectory

    def _exec_traj(self):

        t = time.time()
        trajectory = self._active_trajectory

        try:
            if t < trajectory["starttime"]:
                return
            
            if trajectory["starttime"] + trajectory["points"][-1]["time_from_start"] < t:
                #trajectory execution is over!
                self._active_trajectory = None
                # TODO: check here the final pose match the last point pose
                self.completed(status.SUCCESS, None)

            for p in trajectory["points"]:
                end = trajectory["starttime"] + p["time_from_start"]
                if "started" in p and t < end:
                    # currently going to this waypoint: fine!
                    break

                elif "started" not in p and t < end:
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

                elif "started" not in p and t > end:
                    logger.warning("Skipped a waypoint on armature <%s>. Wrong 'time_from_start'?" % self.name())

                # case: "started" and t > end: do nothing, go to next point
        except KeyError as ke:
            self._active_trajectory = None
            self.completed(status.FAILED, "Error: invalid trajectory: key %s was expected." % ke)

    def interrupt(self):
    
        for joint in self.local_data.keys():
            self.local_data[joint] = self._get_joint_value(joint)
            if joint in self.joint_speed:
                del self.joint_speed[joint]

        self._active_trajectory = None

        super(Armature,self).interrupt()

    @async_service
    def set_target(self,x ,y, z):
        """
        Sets a target position for the armature's tip.

        MORSE uses inverse kinematics to find the joint
        angles/positions in order to get the armature tip as close
        as possible to the target.

        .. important::

            No obstacle avoidance takes place: while moving the armature
            may hit objects.

        .. warning::
            
            Not implemented yet! Only as a placeholder!

        :param x: X coordinate of the IK target
        :param y: Y coordinate of the IK target
        :param z: Z coordinate of the IK target
        """
        raise MorseRPCInvokationError("Not implemented yet!")

    def default_action(self):
        """
        Move the armature according to both the value of local_data

        """

        if self._active_trajectory:
            self._exec_traj()


        armature = self.bge_object

        position_reached = True
        for channel in armature.channels:

            # we assume a joint is prismatic (translation joint) if its IK 'stretch' parameter is non-null
            is_prismatic = self._is_prismatic(channel)

            joint = channel.name

            if joint in self.joint_speed and self.joint_speed[joint]:
                speed = self.joint_speed[joint]
            else:
                speed = self.linear_speed if is_prismatic else self.radial_speed


            # Retrieve the rotation or translation axis
            axis_index = next(i for i, j in enumerate(self.find_dof(channel)) if j)

            if is_prismatic:
                dist = self.local_data[joint] - channel.pose_head[2] # we take the last index ('Z') of the pose of the HEAD of the bone as the absolute translation of the joint. Not 100% sure it is right...
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


