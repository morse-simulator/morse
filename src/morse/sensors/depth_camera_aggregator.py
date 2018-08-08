import logging; logger = logging.getLogger("morse." + __name__)
from morse.core.services import async_service
import morse.core.sensor
from morse.core import blenderapi
from functools import partial
from morse.helpers.components import add_data, add_property

class DepthCameraAggregator(morse.core.sensor.Sensor):

    _name = "Depth Camera Aggregator Unit"

    add_data('points', 'none', 'memoryview', "List of 3D points from the depth "
             "camera. memoryview of a set of float(x,y,z). The data is of size "
             "``(nb_points * 12)`` bytes (12=3*sizeof(float).")
    add_data('nb_points', 0, 'int', "the number of points found in the "
             "points list. It must be inferior to cam_width * cam_height")

    add_property("master_camera", '', 'master_camera', 'string',
                 "the name of master camera : all the points will be expressed "
                 "in the frame of this camera")

    def __init__(self, obj, parent=None):
        """ Constructor method.
        Receives the reference to the Blender object.
        The second parameter should be the name of the object's parent.
        """
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        morse.core.sensor.Sensor.__init__(self, obj, parent)

        # Name of the camera
        self.camera_name_list = []

        # Instance of the camera, will be filled later because as this point,
        # child component are not yet initialized
        self.camera_list = []
        self.master = None

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
            self.camera_name_list.append(camera_name)


        self.converter = None
        self.capturing = True

        logger.info("Depth Camera Aggregator has %d cameras" % len(self.camera_name_list))

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
        self._expected_answer = len(self.camera_list)
        for camera in self.camera_list:
            camera.capture(partial(self.capture_completion), n)

    def sensor_to_robot_position_3d(self):
        return self.master.sensor_to_robot_position_3d()

    def default_action(self):
        """ Main function of this component. """

        # Initialiazation part
        if self.camera_name_list != [] and self.converter is None:
            for camera_name in self.camera_name_list:
                logger.info("Getting instance for %s", camera_name)
                self.camera_list.append(blenderapi.persistantstorage().componentDict[camera_name])

            logger.info("Getting master for %s", self.master_camera)
            self.master = blenderapi.persistantstorage().componentDict[self.master_camera]

            max_nb_points = 0
            for camera in self.camera_list:
                max_nb_points += camera.image_width * camera.image_height * 3

            from morse.sensors.depthaggregator import Aggregator
            self.converter = Aggregator(max_nb_points)

        # Prepare list to send
        images = []
        for i, camera in enumerate(self.camera_list):
            transfo = self.master.position_3d.transformation3d_with(camera.position_3d)
            image = camera.pts
            if image is not None:
                images.append((transfo.x, 
                               transfo.y, 
                               transfo.z, 
                               transfo.roll, 
                               transfo.pitch, 
                               transfo.yaw,
                               image))

        pts = self.converter.merge(images)
        self.local_data['points'] = pts;
        self.local_data['nb_points'] = int(len(pts) / 12)

