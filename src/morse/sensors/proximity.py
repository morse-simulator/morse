import logging; logger = logging.getLogger("morse." + __name__)
import GameLogic
import morse.core.sensor

class ProximitySensorClass(morse.core.sensor.MorseSensorClass):
    """ Distance sensor to detect nearby robots """

    def __init__(self, obj, parent=None):
        """ Constructor method.

        Receives the reference to the Blender object.
        The second parameter should be the name of the object's parent.
        """
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(obj, parent)

        self.local_data['near_robots'] = {}
        try:
            self._range = self.blender_obj['Range']
        except KeyError:
            # Set a default range of 100m
            self._range = 100
            
        logger.info('Component initialized')


    def default_action(self):
        """ Create a list of robots within a certain radius of the sensor. """

        self.local_data['near_robots'] = {}

        parent = self.robot_parent.blender_obj

        # Get the fire sources
        for robot in GameLogic.robotDict.keys():
            # Skip distance to self
            if parent != robot:
                distance = self._measure_distance_to_robot (parent, robot)
                if distance <= self._range:
                    self.local_data['near_robots'][robot.name] = distance



    def _measure_distance_to_robot(self, own_robot, target_robot):
        """ Compute the distance between two robots

        Parameters are two blender objects
        """
        distance, globalVector, localVector = own_robot.getVectTo(target_robot)
        logger.debug("Distance from robot {0} to robot {1} = {2}".format(own_robot, target_robot, distance))
        return distance
