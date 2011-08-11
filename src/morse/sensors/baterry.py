import logging; logger = logging.getLogger("morse." + __name__)
import bge
import morse.core.sensor
import time

class BatteryClass(morse.core.sensor.MorseSensorClass):
    """ Class definition for the battery sensor.
        Sub class of Morse_Object. 

        DischargingRate: float in percent-per-seconds (game-property)
    """

    def __init__(self, obj, parent=None):
        """ Constructor method.
            Receives the reference to the Blender object.
            The second parameter should be the name of the object's parent. """
        logger.info("%s initialization" % obj.name)
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(obj, parent)

        self.local_data['charge'] = 100.0
        self._time = time.clock()

        logger.info('Component initialized')

    def default_action(self):
        """ Main function of this component. """
        newtime = time.clock()
        charge = self.local_data['charge']
        dt = newtime - self._time

        if self.isInChargingZone() and charge < 100:
            charge = charge + (dt * self.blender_obj['DischargingRate'])
        elif charge > 0:
            charge = charge - (dt * self.blender_obj['DischargingRate'])

        # Store the data acquired by this sensor that could be sent
        #  via a middleware.
        self.local_data['charge'] = float(charge)
        # update the current time
        self._time = newtime

    def isInChargingZone(self):
        # Test if the robot (parent) is in a charging zone
        pose = self.position_3d
        # look for a charging zon in the scene
        # TODO for 'charging_zone' in scene:
        # if the robot is near the zone, return true
        return False

