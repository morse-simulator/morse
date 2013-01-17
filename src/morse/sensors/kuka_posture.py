import logging; logger = logging.getLogger("morse." + __name__)
import morse.core.sensor
from morse.core import blenderapi

class KukaPostureClass(morse.core.sensor.Sensor):
    """ KUKA posture sensor
        
    Reads the position of the KUKA LWR arm with respect to the robot,
    as well as the angles of each of the segments."""

    def __init__(self, obj, parent=None):
        """ Constructor method.

        Receives the reference to the Blender object.
        The second parameter should be the name of the object's parent.
        """
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(obj, parent)

        self._kuka_armature = None
        # Check if robot parent has a child with the name specified
        #  in this objects property 'KUKAname'.
        # By default it will look for "kuka_armature"
        armature_name = "kuka_armature"
        if 'KUKAname' in self.bge_object:
            armature_name = self.bge_object['KUKAname']
        for child in self.robot_parent.bge_object.children:
            if str(child) == armature_name:
                self._kuka_armature = child
                break

        if self._kuka_armature:
            logger.debug("Found kuka_arm: '%s'" % self._kuka_armature.name)
        else:
            logger.warning("Kuka arm not found. The 'kuka_posture' sensor will do nothing")
            return

        # The Kuka actuator might not be instantiated yet
        self._kuka_actuator_instance = None

        # Define the variables in 'local_data'
        self.local_data['x'] = 0.0
        self.local_data['y'] = 0.0
        self.local_data['z'] = 0.0
        self.local_data['yaw'] = 0.0
        self.local_data['pitch'] = 0.0
        self.local_data['roll'] = 0.0
        for channel in self._kuka_armature.channels:
            self.local_data[channel.name] = 0.0

        logger.info('Component initialized')

    def get_kuka_actuator_instance(self):
        # Get the reference to the class instance of the kuka actuator
        component_dict = blenderapi.persistantstorage().componentDict
        if self._kuka_armature and self._kuka_armature.name in component_dict:
            self._kuka_actuator_instance = component_dict[self._kuka_armature.name]

    def default_action(self):
        """ Get the x, y, z, yaw, pitch and roll of the KUKA armature,
        and the rotation angle for each of the segments. """
        if not self._kuka_armature:
            return
        if not self._kuka_actuator_instance:
            self.get_kuka_actuator_instance()

        # Take the base position from the kuka arm actuator
        self.local_data['x'] = self._kuka_actuator_instance.position_3d.x
        self.local_data['y'] = self._kuka_actuator_instance.position_3d.y
        self.local_data['z'] = self._kuka_actuator_instance.position_3d.z
        self.local_data['yaw'] = self._kuka_actuator_instance.position_3d.yaw
        self.local_data['pitch'] = self._kuka_actuator_instance.position_3d.pitch
        self.local_data['roll'] = self._kuka_actuator_instance.position_3d.roll

        # Check all the segments of the armature
        for channel in self._kuka_armature.channels:
            segment_angle = channel.joint_rotation

            # Use the corresponding direction for each rotation
            #  taken directly from the armature instance
            if self._kuka_actuator_instance._dofs[channel.name][1] == 1:
                self.local_data[channel.name] = segment_angle[1]
            elif self._kuka_actuator_instance._dofs[channel.name][2] == 1:
                self.local_data[channel.name] = segment_angle[2]
