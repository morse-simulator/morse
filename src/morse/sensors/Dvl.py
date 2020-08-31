import logging; logger = logging.getLogger("morse." + __name__)
import morse.core.sensor
from morse.core.services import service, async_service
from morse.core import status
from morse.helpers.components import add_data, add_property
from morse.core import blenderapi
from math import degrees

class DVL(morse.core.sensor.Sensor):

    _name = "DVL"
    _short_desc = "Sensor providing bottom referenced velocities in addition to altitude."

    # define here the data fields exported by your sensor
    # format is: field name, default initial value, type, description
    add_data('Bx',0.0, "float", 'Surge velocity in body frame')
    add_data('By',0.0, "float", 'Sway velocity in body frame')
    add_data('Bz',0.0, "float", 'Heave velocity in body frame')
    add_data('Wx',0.0, "float", 'Surge velocity in inertial frame')
    add_data('Wy',0.0, "float", 'Sway velocity in inertial frame')
    add_data('Wz',0.0, "float", 'Heave velocity in inertial frame')
    add_data('heading',0.0, "float", 'Compass heading in degrees')
    add_data('depth',0.0, "float", 'Depth in m (increasing downwards)')
    add_data('z', 0.0, "float", 'Distance to bottom')

    add_property('_magnetic_offset', 0.0, "MagOffset", "float",
                 "Magnetic offset from true north at current location") 
    add_property('_max_range', 100.0, "MaxRange", "float", 
                 "Maximum distance to which ground is detected."
                 "If nothing is detected, return +infinity")

    def __init__(self, obj, parent=None):
        logger.info("%s initialization" % obj.name)
        # Call the constructor of the parent class
        morse.core.sensor.Sensor.__init__(self, obj, parent)

        # Do here sensor specific initializations
        logger.info('Component initialized, runs at %.2f Hz', self.frequency)

    def default_action(self):
        """ Main loop of the sensor.

        Implements the component behaviour
        """
        # Get the parent robot
        robot = self.robot_parent

        # Linear velocities in world frame
        WorldVel = robot.bge_object.getLinearVelocity(False)

        self.local_data['Wx'] = WorldVel.x # Surge
        self.local_data['Wy'] = WorldVel.y # Sway
        self.local_data['Wz'] = WorldVel.z # Heave
        
        # Linear velocities in body frame
        BodyVel = robot.bge_object.getLinearVelocity(True)

        self.local_data['Bx'] = BodyVel.x # Surge
        self.local_data['By'] = BodyVel.y # Sway
        self.local_data['Bz'] = BodyVel.z # Heave

        # Instantaneous yaw of vehicle with 'Y' axis pointing North
        yaw_degrees = degrees(self.position_3d.yaw)-90.0

        # Heading is negative yaw
        if yaw_degrees > 0.0:
            heading = 360.0-yaw_degrees
        else:
            heading = -yaw_degrees

        # Subtract magnetic offset (out of simulator you add it)
        heading -= self._magnetic_offset;

        # and check again...
        if heading < 0.0:
            heading = 360.0+heading
       
        self.local_data['heading'] = heading

        # Vehicle depth
        self.local_data['depth'] = -self.position_3d.z

        # Figure out altitude above terrain
        target = self.position_3d.translation
        target[2] -= 1.0

        # Code from the radar altimeter sensor to get altitude 
        _, point, _ = self.bge_object.rayCast(target, None, self._max_range)
        logger.debug("Echo sounder points to %s and hits %s" % (target, point))
        if point:
            self.local_data['z'] = self.bge_object.getDistanceTo(point)
        else:
            self.local_data['z'] = float('inf')