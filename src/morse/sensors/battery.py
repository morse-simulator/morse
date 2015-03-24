import logging; logger = logging.getLogger("morse." + __name__)
import morse.core.sensor
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

    If the battery enters in a **Charging zone**, the battery will
    gradually recharge.
    """

    _name = "Battery Sensor"

    add_property('_discharging_rate', 0.05, 'DischargingRate', "float",
                  "Battery discharging rate, in percent per seconds")

    add_data('charge', 100.0, "float", "Initial battery level, in percent")
    add_data('status', "Charged", "string", "Charging Status")

    def __init__(self, obj, parent=None):
        """ Constructor method.
            Receives the reference to the Blender object.
            The second parameter should be the name of the object's parent. """
        logger.info("%s initialization" % obj.name)
        # Call the constructor of the parent class
        morse.core.sensor.Sensor.__init__(self, obj, parent)

        self._time = self.robot_parent.gettime()

        logger.info('Component initialized, runs at %.2f Hz', self.frequency)

    def default_action(self):
        """ Main function of this component. """
        charge = self.local_data['charge']
        dt = self.robot_parent.gettime() - self._time 

        if self.in_zones(type = 'Charging'):
            charge = charge + dt * self._discharging_rate
            status = "Charging"
            if charge > 100.0:
                charge = 100.0
                status = "Charged"
        else:
            charge = charge - dt * self._discharging_rate
            status = "Discharging"
            if charge < 0.0:
                charge = 0.0

        # Store the data acquired by this sensor that could be sent
        #  via a middleware.
        self.local_data['charge'] = float(charge)
        self.local_data['status'] = status
        # update the current time
        self._time = self.robot_parent.gettime()
