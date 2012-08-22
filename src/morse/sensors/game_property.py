import logging; logger = logging.getLogger("morse." + __name__)
import bge
import morse.core.sensor

class GamePropertyClass(morse.core.sensor.MorseSensorClass):
    """ Game Property sensor """

    def __init__(self, obj, parent=None):
        """ Constructor method.

        Receives the reference to the Blender object.
        The second parameter should be the name of the object's parent.
        """
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(obj, parent)
               
        if self.blender_obj['Names_Of_Properties'] == '':
            properties = self.robot_parent.blender_obj.getPropertyNames()            
        else:
            properties = self.blender_obj['Names_Of_Properties'].split(',')

        for property in properties:
            self.local_data[property.strip()] = 0.0
        
        logger.info('Component initialized')


    def default_action(self):
        """ Get the Game property values from parent object.
        
        """

        for key in self.local_data:
            self.local_data[key] = self.robot_parent.blender_obj[key]
