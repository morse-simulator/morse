import logging; logger = logging.getLogger("morse." + __name__)
import bge
import morse.core.sensor

class PropertyClass(morse.core.sensor.MorseSensorClass):
    """ Game Property sensor """

    def __init__(self, obj, parent=None):
        """ Constructor method.

        Receives the reference to the Blender object.
        The second parameter should be the name of the object's parent.
        """
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(obj, parent)
        
        self.add_property('_names_of_properties', '', 'NamesOfProperties')
        
        logger.info('Component initialized')

    """
    def first_action(self):

        if self._names_of_properties == '':
            properties = self.robot_parent.blender_obj.getPropertyNames()            
        else:
            properties = self._names_of_properties.split(',')

        for property in properties:
            self.local_data[property.strip()] = 0.0
    """

    def default_action(self):
        """ Get the property values from parent object.
        
        """
        if self._names_of_properties == '':
            properties = self.robot_parent.blender_obj.getPropertyNames()            
        else:
            properties = self._names_of_properties.split(',')

        for property in properties:
            self.local_data[property.strip()] = self.robot_parent.blender_obj[property.strip()]
        """
        for key in self.local_data:
            self.local_data[key] = self.robot_parent.blender_obj[key]
        """
