import logging; logger = logging.getLogger("morse." + __name__)
import bge
import morse.core.actuator

class DestinationActuatorClass(morse.core.actuator.MorseActuatorClass):
    """ Destination motion controller

    This controller will receive a destination point and
    make the robot move to that location by moving without turning.
    """

    def __init__(self, obj, parent=None):

        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(obj, parent)

        self._tolerance = 0.5
        self._speed = 5.0
        self.destination = self.blender_obj.position

        #self.local_data['speed'] = 0.0
        self.local_data['x'] = self.destination[0]
        self.local_data['y'] = self.destination[1]
        self.local_data['z'] = self.destination[2]

        logger.info('Component initialized')


    def default_action(self):
        """ Move the object towards the destination. """
        parent = self.robot_parent

        self.destination = [ self.local_data['x'], self.local_data['y'], self.local_data['z'] ]

        logger.debug("STRAIGHT GOT DESTINATION: {0}".format(self.destination))
        logger.debug("Robot {0} move status: '{1}'".format(parent.blender_obj.name, parent.move_status))

        # Vectors returned are already normalised
        distance, global_vector, local_vector = self.blender_obj.getVectTo(self.destination)

        logger.debug("My position: {0}".format(self.blender_obj.position))
        logger.debug("GOT DISTANCE: {0}".format(distance))
        logger.debug("Global vector: {0}".format(global_vector))
        logger.debug("Local  vector: {0}".format(local_vector))

        if distance > self._tolerance:
            # Set the robot status
            parent.move_status = "Transit"

            # Tick rate is the real measure of time in Blender.
            # By default it is set to 60, regardles of the FPS
            # If logic tick rate is 60, then: 1 second = 60 ticks
            ticks = bge.logic.getLogicTicRate()
    
            # Scale the speeds to the time used by Blender
            try:
                vx = global_vector[0] * self._speed / ticks
                vy = global_vector[1] * self._speed / ticks
                vz = global_vector[2] * self._speed / ticks
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
            logger.debug("Robot {0} move status: '{1}'".format(parent.blender_obj.name, parent.move_status))

        # Give the movement instructions directly to the parent
        # The second parameter specifies a "local" movement
        parent.blender_obj.applyMovement([vx, vy, vz], False)
        #parent.blender_obj.applyRotation([rx, ry, rz], False)
