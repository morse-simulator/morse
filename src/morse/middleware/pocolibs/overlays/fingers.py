import logging; logger = logging.getLogger("morse." + __name__)
logger.setLevel(logging.DEBUG)

from morse.core.services import service
from morse.core.overlay import MorseOverlay

from morse.middleware.pocolibs.sensors.General_Poster.ors_poster import new_poster

class Fingers(MorseOverlay):
    def __init__(self, overlaid_object):
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(overlaid_object)
        new_poster("fingerCntrl", 4)

    def name(self):
        return "fingers"

    @service
    def Init(self, grasped, mode):
        pass

    @service
    def OpenGrip(self):
        self.overlaid_object.release()

    @service
    def CloseGrip(self):
        grasped = self.overlaid_object.grab()
        if grasped:
            return [1]
        else:
            return [0]



