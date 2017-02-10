import logging; logger = logging.getLogger("morse." + __name__)
import GameLogic
import math
import morse.core.sensor

class DynamicIMUClass(morse.core.sensor.MorseSensorClass):
    """ IMU sensor """

    def __init__(self, obj, parent=None):
        """ Constructor method.

        Receives the reference to the Blender object.
        The second parameter should be the name of the object's parent.
        """
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(obj, parent)
        
        # Variables to store the previous velocities
        self.pvDynamic = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Tick rate is the real measure of time in Blender.
        # By default it is set to 60, regardles of the FPS
        # If logic tick rate is 60, then: 1 second = 60 ticks
        self.ticksDynamic = GameLogic.getLogicTicRate()
        
        self.local_data['distanceDynamic'] = 0.0
        self.local_data['velocityDynamic'] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.local_data['accelerationDynamic'] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        logger.info('Component initialized')


    def default_action(self):
        """ Get the distance, velocity and compute acceleration of the blender object.
        
        The acceleration is computed using the blender tics
        to measure time.
        Since the time in Blender is computed with N ticks equaling a second,
        this method will be called N times each second, and so the time
        between calls is 1/N.
        When computing velocity as v = d / t, and t = 1 / N, then
        v = d * N
        where N is the number of ticks
        """
        # Prepare varibles
        v = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        a = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
              
        # Get the Blender object of the parent robot
        parent = self.robot_parent.blender_obj
        
        # Calculate distance
        distance = math.sqrt(parent.localPosition.x**2 + parent.localPosition.y**2 + parent.localPosition.z**2)
        logger.debug("DISTANCE: %.4f" % distance)        
        
        # Get speed
        v[0] = parent.localLinearVelocity.x
        v[1] = parent.localLinearVelocity.y
        v[2] = parent.localLinearVelocity.z
        v[3] = parent.localAngularVelocity.x
        v[4] = parent.localAngularVelocity.y
        v[5] = parent.localAngularVelocity.z
        logger.debug("SPEED: (%.4f, %.4f, %.4f, %4f, %4f, %4f)" % (v[0], v[1], v[2], v[3], v[4], v[5]))

        # Calculate acceleration from speed
        a[0] = (v[0] - self.pvDynamic[0]) * self.ticksDynamic
        a[1] = (v[1] - self.pvDynamic[1]) * self.ticksDynamic
        a[2] = (v[2] - self.pvDynamic[2]) * self.ticksDynamic
        a[3] = (v[3] - self.pvDynamic[3]) * self.ticksDynamic
        a[4] = (v[4] - self.pvDynamic[4]) * self.ticksDynamic
        a[5] = (v[5] - self.pvDynamic[5]) * self.ticksDynamic
        logger.debug("ACCELERATION: (%.4f, %.4f, %.4f, %4f, %4f, %4f)" % (a[0], a[1], a[2], a[3], a[4], a[5]))

        # Update the data for the velocity
        self.pvDynamic = v

        # Store the data acquired by this sensor that could be sent
        #  via a middleware.
        self.local_data['distanceDynamic'] = distance
        self.local_data['velocityDynamic'] = v
        self.local_data['accelerationDynamic'] = a
