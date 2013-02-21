import logging; logger = logging.getLogger("morse." + __name__)

from morse.core import blenderapi
import morse.core.sensor
from morse.core.services import service
from morse.helpers.components import add_data, add_property

class Proximity(morse.core.sensor.Sensor):
    """
    This sensor can be used to determine which other objects are within a
    certain radius of the sensor. It performs its test based only on distance.
    The type of tracked objects can be specified using the **Track** property.
    """

    _name = "Proximity Sensor"
    _short_desc = "Distance sensor to detect nearby objects."

    add_data('near_objects', {}, "dict", "A list of the tracked objects located "
            "within the given radius. The keys of the dictionary are the object "
            "names, and the values are the distances (in meters) from the "
            "sensor.")
    add_data('near_robots', {}, "dict", "deprecated. Points to near_objects for compatibility reasons.")

    add_property('_range', 100, 'Range', "float", "The distance, in meters "
            "beyond which this sensor is unable to locate other robots.")
    add_property('_tag', "Robot_Tag",  'Track',  "string",  "The type of "
            "tracked objects. This type is looked for as game property of scene "
            "objects. You must then add a new game property to the objects you "
            "want to be detected by the proximity sensor.")

    def __init__(self, obj, parent=None):
        """ Constructor method.

        Receives the reference to the Blender object.
        The second parameter should be the name of the object's parent.
        """
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(obj, parent)

        
        logger.info('Component initialized')

    @service
    def set_range(self, range):
        """
        The service expects a float range (in meter), and modify the range
        used to detect objects around the proximity sensor.

        :param range: detection range, in meters
        """
        self._range = float(range)

    @service
    def set_tracked_tag(self, tag):
        """
        The service allows to modify the kind of objects detected by the
        proximity sensor.

        :param tag: value of the *Track* property used to select detected
                    objects.
        """
        self._tag = tag

    def default_action(self):
        """ Create a list of tagged objects within a certain radius of the sensor. """

        self.local_data['near_objects'] = {}
        self.local_data['near_robots'] = self.local_data['near_objects']

        parent = self.robot_parent.bge_object

        # Get the tracked sources
        for obj in blenderapi.scene().objects:
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
