import logging; logger = logging.getLogger("morse." + __name__)
import math

from morse.modifiers.abstract_modifier import AbstractModifier

class NEDModifier(AbstractModifier):
    """ Convert between Blender coordinates and NED coordinates. """
    def modify(self):
        pass
    
class CoordinatesToNED(NEDModifier):
    def modify(self):
        """ Convert the coordinates from ENU to NED. """
        try:
            tmp = self.data['x']
            self.data['x'] = self.data['y']
            self.data['y'] = tmp
            self.data['z'] = - self.data['z']
        except KeyError as detail:
            logger.warning("Unable to use %s on component %s. It does not have data %s." 
                           % (self.__class__.__name__, self.component_name, detail))

class CoordinatesFromNED(NEDModifier):        
    def modify(self):
        """ Convert the coordinates from NED to ENU. """
        try:
            tmp = self.data['x']
            self.data['x'] = self.data['y']
            self.data['y'] = tmp
            self.data['z'] = - self.data['z']
        except KeyError as detail:
            logger.warning("Unable to use %s on component %s. It does not have data %s." 
                           % (self.__class__.__name__, self.component_name, detail))

class AnglesFromNED(NEDModifier):
    def modify(self):
        """ Convert the angles from ENU to NED. """
        try:
            roll = math.pi/2 - self.data['yaw']
            self.data['yaw'] = self.data['roll']
            self.data['pitch'] = - self.data['pitch']
            self.data['roll'] = roll
        except KeyError as detail:
            logger.warning("Unable to use %s on component %s. It does not have data %s." 
                           % (self.__class__.__name__, self.component_name, detail))

class AnglesToNED(NEDModifier):
    def modify(self):
        """ Convert the angles from NED to ENU. """
        try:
            yaw = math.pi/2 - self.data['roll']
            self.data['pitch'] = - self.data['pitch']
            self.data['roll'] = self.data['yaw']
            self.data['yaw'] = yaw
        except KeyError as detail:
            logger.warning("Unable to use %s on component %s. It does not have data %s." 
                           % (self.__class__.__name__, self.component_name, detail))
