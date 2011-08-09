import logging; logger = logging.getLogger("morse." + __name__)
import GameLogic
import math
import morse.core.sensor

class AccelerometerClass(morse.core.sensor.MorseSensorClass):
    """ Accelerometer sensor """

    def __init__(self, obj, parent=None):
        """ Constructor method.

        Receives the reference to the Blender object.
        The second parameter should be the name of the object's parent.
        """
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(obj, parent)

        # Variables to store the previous position
        self.ppx = 0.0
        self.ppy = 0.0
        self.ppz = 0.0
        # Variables to store the previous velocity
        self.pvx = 0.0
        self.pvy = 0.0
        self.pvz = 0.0
        # Make a new reference to the sensor position
        self.p = self.blender_obj.position
        self.v = [0.0, 0.0, 0.0]            # Velocity
        self.pv = [0.0, 0.0, 0.0]           # Previous Velocity
        self.a = [0.0, 0.0, 0.0]            # Acceleration

        # Tick rate is the real measure of time in Blender.
        # By default it is set to 60, regardles of the FPS
        # If logic tick rate is 60, then: 1 second = 60 ticks
        self.ticks = GameLogic.getLogicTicRate()

        self.local_data['distance'] = 0.0
        self.local_data['velocity'] = [0.0, 0.0, 0.0]
        self.local_data['acceleration'] = [0.0, 0.0, 0.0]

        logger.info('Component initialized')


    def default_action(self):
        """ Compute the speed and accleration of the robot

        The speed and acceleration are computed using the blender tics
        to measure time.
        Since the time in Blender is computed with N ticks equaling a second,
        this method will be called N times each second, and so the time
        between calls is 1/N.
        When computing velocity as v = d / t, and t = 1 / N, then
        v = d * N
        where N is the number of ticks
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
        self.v[0] = dx * self.ticks
        self.v[1] = dy * self.ticks
        self.v[2] = dz * self.ticks
        logger.debug("SPEED: (%.4f, %.4f, %.4f)" % (self.v[0], self.v[1], self.v[2]))

        self.a[0] = (self.v[0] - self.pvx) * self.ticks
        self.a[1] = (self.v[1] - self.pvy) * self.ticks
        self.a[2] = (self.v[2] - self.pvz) * self.ticks
        logger.debug("ACCELERATION: (%.4f, %.4f, %.4f)" % (self.a[0], self.a[1], self.a[2]))

        # Update the data for the velocity
        self.pvx = self.v[0]
        self.pvy = self.v[1]
        self.pvz = self.v[2]

        # Store the important data
        self.local_data['velocity'] = self.v
        self.local_data['acceleration'] = self.a
