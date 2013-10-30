import logging; logger = logging.getLogger("morse." + __name__)
from abc import ABCMeta
import morse.core.object
from morse.core import blenderapi

class Robot(morse.core.object.Object):
    """ Basic Class for all robots

    Inherits from the base object class.
    """

    # Make this an abstract class
    __metaclass__ = ABCMeta

    def __init__ (self, obj, parent=None):
        """ Constructor method. """
        # Call the constructor of the parent class
        super(Robot, self).__init__(obj, parent)
        
        # Add the variable move_status to the object
        self.move_status = "Stop"

        # shift against the simulator time (in ms)
        self.time_shift = 0.0


    def action(self):
        """ Call the regular action function of the component. """
        # Update the component's position in the world
        self.position_3d.update(self.bge_object)

        self.default_action()

    def gettime(self):
        """ Return the current time, as seen by the robot, in milli seconds """
        return blenderapi.persistantstorage().current_time * 1000.0 +\
               self.time_shift

