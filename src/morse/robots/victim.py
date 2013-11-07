import logging; logger = logging.getLogger("morse." + __name__)
import morse.core.robot
from morse.core.services import service


class Victim(morse.core.robot.Robot):
    """ Class definition for the pseudo-robot that represents
        a human victim. Mainly used for the ROSACE rescue scenario.
        Sub class of Morse_Object. """

    def __init__(self, obj, parent=None):
        """ Constructor method.
            Receives the reference to the Blender object.
            Optionally it gets the name of the object's parent,
            but that information is not currently used for a robot. """
        # Call the constructor of the parent class
        logger.info('%s initialization' % obj.name)
        super(self.__class__,self).__init__(obj, parent)

        if self.bge_object['Injured'] == True:
            #  Set the mesh color to red
            obj.color = [1.0, 0.5, 0.5, 1.0]
        else:
            #  Set the mesh color to red
            obj.color = [1.0, 1.0, 1.0, 1.0]

        # Convert the 'Requirements' property,
        #  from a string to a list of integers
        obj['Requirements'] = [int(x) for x in obj['Requirements'].split(",")]

        logger.info('Component initialized')


    @service
    def heal(self):
        """ Change the status of the victim
        
        Change the material to a green color,
        and the status to healed.
        """
        if self.bge_object['Severity'] > 0:
            self.bge_object['Severity'] -= 1
            # Set the colors depending on the severity of the injuries
            red = 1 - self.bge_object['Severity'] * 0.05
            green = 0.5 + red
            self.bge_object.color = [red, green, 0.5, 1.0]

        # When fully healed, mark as not injured
        if self.bge_object['Severity'] == 0:
            self.bge_object['Injured'] = False


    def default_action(self):
        """ Main function of this component. """
        pass
