import logging; logger = logging.getLogger("morse." + __name__)
import morse.core.actuator
from morse.helpers.components import add_data, add_property

class Destination(morse.core.actuator.Actuator):
    """
    This actuator reads the coordinates of a destination point, and moves the robot
    in a straight line towards the given point, without turning.  It provides a
    very simplistic movement, and can be used for testing or for robots with
    holonomic movement.  The speeds provided are internally adjusted to the Blender
    time measure.
    """

    _name = "Destination"
    _short_desc = "Instruct the robot to move towards a given target"

    add_data('x', 'current X pos', "float", "X coordinate of the destination")
    add_data('y', 'current Y pos', "float", "Y coordinate of the destination")
    add_data('z', 'current Z pos', "float", "Z coordinate of the destination")

    add_property('_tolerance', 0.5, 'Tolerance')
    add_property('_speed', 5.0, 'Speed')
    add_property('_type', 'Velocity', 'ControlType', 'string',
                 "Kind of control, can be one of ['Velocity', 'Position']")
    add_property('_remain_at_destination', False, 'RemainAtDestination', 'bool',
                "If true (default: false), the robot actively attempts to \
                remain at the destination once reached. This is especially \
                useful for flying robots that would otherwise typically fall.")

    def __init__(self, obj, parent=None):

        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        morse.core.actuator.Actuator.__init__(self, obj, parent)

        self._destination = self.bge_object.position

        #self.local_data['speed'] = 0.0
        self.local_data['x'] = self._destination[0]
        self.local_data['y'] = self._destination[1]
        self.local_data['z'] = self._destination[2]

        logger.info('Component initialized')


    def default_action(self):
        """ Move the object towards the destination. """
        parent = self.robot_parent

        self._previous_destination = self._destination

        self._destination = [ self.local_data['x'], self.local_data['y'], self.local_data['z'] ]

        # Do nothing at all if:
        # - we were not ask to actively remain at the last wp
        # - we already are at destination
        # - no new destination has been received.
        if not self._remain_at_destination and \
           parent.move_status == "Stop" and \
           self._destination == self._previous_destination:
               return

        logger.debug("STRAIGHT GOT DESTINATION: {0}".format(self._destination))
        logger.debug("Robot {0} move status: '{1}'".format(parent.bge_object.name, parent.move_status))

        # Vectors returned are already normalised
        distance, global_vector, local_vector = self.bge_object.getVectTo(self._destination)

        logger.debug("My position: {0}".format(self.bge_object.position))
        logger.debug("GOT DISTANCE: {0}".format(distance))
        logger.debug("Global vector: {0}".format(global_vector))
        logger.debug("Local  vector: {0}".format(local_vector))

        if distance > self._tolerance:
            # Set the robot status
            parent.move_status = "Transit"
    
            # Scale the speeds to the time used by Blender
            try:
                if self._type == 'Position':
                    vx = global_vector[0] * self._speed / self.frequency
                    vy = global_vector[1] * self._speed / self.frequency
                    vz = global_vector[2] * self._speed / self.frequency
                else:
                    vx = global_vector[0] * self._speed
                    vy = global_vector[1] * self._speed
                    vz = global_vector[2] * self._speed
            # For the moment ignoring the division by zero
            # It happens apparently when the simulation starts
            except ZeroDivisionError:
                pass

        # If the target has been reached, change the status
        else:
            # Reset movement variables
            vx, vy, vz = 0.0, 0.0, 0.0
            #rx, ry, rz = 0.0, 0.0, 0.0

            parent.move_status = "Stop"
            logger.debug("TARGET REACHED")
            logger.debug("Robot {0} move status: '{1}'".format(parent.bge_object.name, parent.move_status))

        
        self.robot_parent.apply_speed(self._type, [vx, vy, vz], [0, 0, 0])
