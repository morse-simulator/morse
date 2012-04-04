import logging; logger = logging.getLogger("morse." + __name__)
import bge

import morse.core.sensor
from morse.core.services import service

class ProximitySensorClass(morse.core.sensor.MorseSensorClass):
    """ Distance sensor to detect nearby objects. """

    def __init__(self, obj, parent=None):
        """ Constructor method.

        Receives the reference to the Blender object.
        The second parameter should be the name of the object's parent.
        """
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(obj, parent)

        self.local_data['near_objects'] = {}
        self.local_data['near_robots'] = {}
        
        try:
            self._range = self.blender_obj['Range']
        except KeyError:
            # Set a default range of 100m
            self._range = 100
        
        try:
            self._tag = self.blender_obj['Track']
        except KeyError:
            # Set default tracked objects to 'Robots'
            self._tag = "Robot_Tag"
        
        logger.info('Component initialized')

    @service
    def set_range(self, range):
        self._range = float(range)

    @service
    def set_tracked_tag(self, tag):
        self._tag = tag

    def default_action(self):
        """ Create a list of tagged objects within a certain radius of the sensor. """

        self.local_data['near_objects'] = {}
        self.local_data['near_robots'] = self.local_data['near_objects']

        parent = self.robot_parent.blender_obj

        # Get the tracked sources
        for obj in bge.logic.getCurrentScene().objects:
            try:
                obj[self._tag]
                # Skip distance to self
                if parent != obj:
                    distance = self._measure_distance_to_object (parent, obj)
                    if distance <= self._range:
                        self.local_data['near_objects'][obj.name] = distance
            except KeyError:
                pass

    def _measure_distance_to_object(self, own_robot, target_object):
        """ Compute the distance between two objects

        Parameters are two blender objects
        """
        distance, globalVector, localVector = own_robot.getVectTo(target_object)
        logger.debug("Distance from robot {0} to object {1} = {2}".format(own_robot, target_object, distance))
        return distance
