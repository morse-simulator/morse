import logging; logger = logging.getLogger("morse." + __name__)
import math
import morse.core.sensor
from morse.helpers.components import add_data

class Accelerometer(morse.core.sensor.Sensor):
    """ 
    This sensor emulates an Accelerometer/Podometer, measuring the
    distance that a robot has moved, the current speed and current
    acceleration. Measurements are done for the 3 axes (X, Y, Z) for
    velocity and acceleration. The values for velocity and acceleration
    are measured at each tic of the Game Engine, measuring the
    difference in distance from the previous tic, and the estimated time
    between tics (60 tics per second is the default in Blender).
    """

    _name = "Accelerometer"

    add_data('distance', 0.0, "float", 
             'distance travelled since the last tick, in meter')
    add_data('velocity', [0.0, 0.0, 0.0], "vec3<float>", 
             'Instantaneous speed in X, Y, Z, in meter sec^-1')
    add_data('acceleration', [0.0, 0.0, 0.0], "vec3<float>", 
             'Instantaneous acceleration in X, Y, Z, in meter sec^-2')

    def __init__(self, obj, parent=None):
        """ Constructor method.

        Receives the reference to the Blender object.
        The second parameter should be the name of the object's parent.
        """
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__, self).__init__(obj, parent)

        # Variables to store the previous position
        self.ppx = 0.0
        self.ppy = 0.0
        self.ppz = 0.0
        # Variables to store the previous velocity
        self.pvx = 0.0
        self.pvy = 0.0
        self.pvz = 0.0
        # Make a new reference to the sensor position
        self.p = self.bge_object.position
        self.v = [0.0, 0.0, 0.0]            # Velocity
        self.pv = [0.0, 0.0, 0.0]           # Previous Velocity
        self.a = [0.0, 0.0, 0.0]            # Acceleration

        logger.info('Component initialized, runs at %.2f Hz', self.frequency)


    def default_action(self):
        """ Compute the speed and accleration of the robot

        The speed and acceleration are computed using the blender tics
        to measure time.
        When computing velocity as v = d / t, and t = 1 / frequency, then
        v = d * frequency
        where frequency is computed from the blender tics and number of skipped
        logic steps for this sensor.
        """
        # Compute the difference in positions with the previous loop
        dx = self.p[0] - self.ppx
        dy = self.p[1] - self.ppy
        dz = self.p[2] - self.ppz
        self.local_data['distance'] = math.sqrt(dx**2 + dy**2 + dz**2)
        logger.debug("DISTANCE: %.4f" % self.local_data['distance'])

        # Store the position in this instant
        self.ppx = self.p[0]
        self.ppy = self.p[1]
        self.ppz = self.p[2]

        # Scale the speeds to the time used by Blender
        self.v[0] = dx * self.frequency
        self.v[1] = dy * self.frequency
        self.v[2] = dz * self.frequency
        logger.debug("SPEED: (%.4f, %.4f, %.4f)" %
                    (self.v[0], self.v[1], self.v[2]))

        self.a[0] = (self.v[0] - self.pvx) * self.frequency
        self.a[1] = (self.v[1] - self.pvy) * self.frequency
        self.a[2] = (self.v[2] - self.pvz) * self.frequency
        logger.debug("ACCELERATION: (%.4f, %.4f, %.4f)" %
                     (self.a[0], self.a[1], self.a[2]))

        # Update the data for the velocity
        self.pvx = self.v[0]
        self.pvy = self.v[1]
        self.pvz = self.v[2]

        # Store the important data
        self.local_data['velocity'] = self.v
        self.local_data['acceleration'] = self.a
