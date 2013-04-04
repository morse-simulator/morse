import logging; logger = logging.getLogger("morse." + __name__)
import morse.core.sensor
import time
from morse.helpers.components import add_data, add_property

class Battery(morse.core.sensor.Sensor):
    """
    This sensor emulates the remaining charge of a battery on the robot.
    It is meant to be used only as an informative measure, to be taken in
    consideration by the planning algorithms. It does not prevent the robot
    from working.

    The charge of the battery decreases with time, using a predefined
    **Discharge rate** specified as a property of the Blender object.
    This rate is independent of the actions performed by the robot, and
    only dependant on the time elapsed since the beginning of the simulation.

    A planned feature is to allow for designated **Charging Zones** where
    the battery will gradually recharge. However, this is not implemented yet.
    """

    _name = "Battery Sensor"

    add_property('_discharging_rate', 0.05, 'DischargingRate', "float", "Battery discharging rate, in percent per seconds")

    add_data('charge', 100.0, "float", "Initial battery level, in percent")

    def __init__(self, obj, parent=None):
        """ Constructor method.
            Receives the reference to the Blender object.
            The second parameter should be the name of the object's parent. """
        logger.info("%s initialization" % obj.name)
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(obj, parent)

        self._time = time.time()

        logger.info('Component initialized, runs at %.2f Hz', self.frequency)

    def default_action(self):
        """ Main function of this component. """
        newtime = time.time()
        charge = self.local_data['charge']
        dt = newtime - self._time

        if self.isInChargingZone() and charge < 100:
            charge = charge + dt * self._discharging_rate
            if charge > 100.0:
                charge = 100.0
        elif charge > 0:
            charge = charge - dt * self._discharging_rate
            if charge < 0.0:
                charge = 0.0

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

