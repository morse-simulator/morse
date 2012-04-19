import logging; logger = logging.getLogger("morse." + __name__)
from abc import ABCMeta, abstractmethod
from morse.core.abstractobject import MorseAbstractObject

import morse.helpers.transformation
from morse.core.services import service


class MorseObjectClass(MorseAbstractObject):
    """ Basic Class for all 3D objects (components) used in the simulation.
        Provides common attributes.
    """

    # Make this an abstract class
    __metaclass__ = ABCMeta

    def __init__ (self, obj, parent=None):

        super(MorseObjectClass,self).__init__()

        # Fill in the data sent as parameters
        self.blender_obj = obj
        self.robot_parent = parent

        # Variable to indicate the activation status of the component
        self._active = True

        # Define the position of sensors with respect
        #  to their robot parent
        # TODO: implement this using morse.helpers.transformation
        if not parent == None:
            self.relative_position = obj.getVectTo(parent.blender_obj)

        # Create an instance of the 3d transformation class
        self.position_3d = morse.helpers.transformation.Transformation3d(obj)

    def __del__(self):
        """ Destructor method. """
        logger.info("%s: I'm dying!!" % self.name())

    def name(self):
        return self.blender_obj.name

    def action(self):
        """ Call the regular action function of the component.

        Can be redefined in some of the subclases (sensor and actuator).
        """
        self.default_action()

    @abstractmethod
    def default_action(self):
        """ Base action performed by any object.

        This method should be implemented by all subclasses that
        will be instanced (GPS, v_Omega, ATRV, etc.).
        """
        pass

    def toggle_active(self):
        self._active = not self._active
