import logging; logger = logging.getLogger("morse." + __name__)
from morse.core.services import async_service
import morse.core.sensor
import GameLogic
from functools import partial

class StereoUnitClass(morse.core.sensor.MorseSensorClass):
    """ Base for stereo pairs

    It is used to link two camera objects, and export the images
    as a stereo pair.
    """

    def __init__(self, obj, parent=None):
        """ Constructor method.
        Receives the reference to the Blender object.
        The second parameter should be the name of the object's parent.
        """
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(obj, parent)

        self.num_cameras = 0
        self.camera_list = []
        # Create a list of the cameras attached to this component
        for child in obj.children:
            # Skip this object if it is not a component
            # It is most likely just a geometric shape object
            try:
                child['Component_Tag']
            except KeyError as detail:
                continue

            camera_name = child.name
            # Store only the name of the camera
            # All data from the camera can be accessed later
            #  by using GameLogic.componentDict[camera_name],
            #  which will return the instance of the camera object
            self.camera_list.append(camera_name)
            self.num_cameras += 1

        logger.info("Stereo Unit has %d cameras" % self.num_cameras)

        logger.info('Component initialized')

    def capture_completion(self, answer):
        self._expected_answer-= 1
        if self._expected_answer == 0:
            status, res = answer
            self.completed(status, res)

    def interrupt(self):
        for camera in self.camera_list:
            camera_instance = GameLogic.componentDict[camera]
            camera_instance.interrupt()

    @async_service
    def capture(self, n):
        self._expected_answer = self.num_cameras
        for camera in self.camera_list:
            camera_instance = GameLogic.componentDict[camera]
            camera_instance.capture(partial(self.capture_completion), n)

    def default_action(self):
        """ Main function of this component. """
        pass
