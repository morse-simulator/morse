import logging; logger = logging.getLogger("morse." + __name__)
from morse.core.services import async_service
import morse.core.sensor
from morse.core import blenderapi
from functools import partial

class StereoUnit(morse.core.sensor.Sensor):
    """
    The purpose of this component is to link together one or more
    cameras, and provide them with the possibility to move together as a
    single unit. It will also provide the connection interface to use
    the information of the cameras attached to it. In the case of two
    cameras, it will provide the stereo information generated from the
    two camera images.

    Related components
    ------------------

    A stereo unit needs to be the parent of one or more :doc:`cameras
    <../sensors/video_camera>`. Otherwise, it does no useful function.

    The movement of the stereo unit is implemented by making it the child of a
    :doc:`Pan-Tilt unit <../actuators/ptu>` actuator.

    Here is an example of how to construct the whole stereo system to mount on top
    of a robot, using the Builder API. Note the order in which components are
    appended to each other, as this is important to get the desired functionality:

    .. code-block:: python

        from morse.builder import *

        # Add a robot
        atrv = ATRV()
        atrv.translate(z=0.1000)
        
        # A pan-tilt unit to be able to orient the cameras
        Platine = PTU()
        Platine.translate(x=0.2000, z=0.9000)
        atrv.append(Platine)
        
        # The STEREO UNIT, where the two cameras will be fixed
        Stereo = StereoUnit()
        Stereo.translate(z=0.0400)
        Platine.append(Stereo)
        
        # Left camera
        CameraL = VideoCamera()
        CameraL.translate(x=0.1000, y=0.2000, z=0.0700)
        Stereo.append(CameraL)
        CameraL.properties(capturing = True)
        CameraL.properties(cam_width = 320)
        CameraL.properties(cam_height = 240)
        CameraL.properties(cam_focal = 25.0000)
        
        # Right camera
        CameraR = VideoCamera()
        CameraR.translate(x=0.1000, y=-0.2000, z=0.0700)
        Stereo.append(CameraR)
        CameraR.properties(capturing = True)
        CameraR.properties(cam_width = 320)
        CameraR.properties(cam_height = 240)
        CameraR.properties(cam_focal = 25.0000)

    """

    _name = "Stereo Camera Unit"

    def __init__(self, obj, parent=None):
        """ Constructor method.
        Receives the reference to the Blender object.
        The second parameter should be the name of the object's parent.
        """
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__, self).__init__(obj, parent)

        self.num_cameras = 0
        self.camera_list = []
        # Create a list of the cameras attached to this component
        for child in obj.children:
            # Skip this object if it is not a component
            # It is most likely just a geometric shape object
            try:
                child['Component_Tag']
            except KeyError:
                continue

            camera_name = child.name
            # Store only the name of the camera All data from the camera
            # can be accessed later by using
            # blenderapi.persistantstorage().componentDict[camera_name],
            # which will return the instance of the camera object
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
            camera_instance = blenderapi.persistantstorage().componentDict[camera]
            camera_instance.interrupt()

    @async_service
    def capture(self, n):
        """
        The service takes an integer an argument and dispatch the call
        to all its individual cameras. The service ends when each camera
        has terminated its work.

        :param n: the number of call to each individual camera
        """
        self._expected_answer = self.num_cameras
        for camera in self.camera_list:
            camera_instance = blenderapi.persistantstorage().componentDict[camera]
            camera_instance.capture(partial(self.capture_completion), n)

    def default_action(self):
        """ Main function of this component. """
        pass
