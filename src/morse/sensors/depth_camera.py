import logging; logger = logging.getLogger("morse." + __name__)
from morse.core.services import async_service
from morse.core import status, mathutils
import morse.core.blenderapi
from morse.sensors.camera import CameraClass
from morse.sensors.video_camera import VideoCameraClass
from morse.helpers.components import add_data
from morse.sensors.zbufferto3d import ZBufferTo3D
from morse.sensors.zbuffertodepth import ZBufferToDepth


class DepthCameraClass(VideoCameraClass):
    """
    This sensor generates a 3D point cloud from the camera perspective.
    """

    _name = "Depth (XYZ) camera"

    add_data('points', 'none', 'memoryview', "List of 3D points from the depth "
             "camera. memoryview of a set of float(x,y,z). The data is of size "
             "``(cam_width * cam_height * 12)`` bytes (12=3*sizeof(float).")

    def __init__(self, obj, parent=None):
        """ Constructor method.

        Receives the reference to the Blender object.
        The second parameter should be the name of the object's parent.
        """
        # Call the constructor of the VideoCameraClass class
        VideoCameraClass.__init__(self, obj, parent)

        # Store the camera parameters necessary for image processing
        self.converter = ZBufferTo3D(self.local_data['intrinsic_matrix'][0][0],\
                                     self.local_data['intrinsic_matrix'][1][1],\
                                     self.near_clipping, self.far_clipping, \
                                     self.image_width, self.image_height)

    def default_action(self):
        """ Update the texture image. """
        # Grab an image from the texture
        if self.bge_object['capturing'] and (self._n != 0) :

            # Call the action of the CameraClass class
            CameraClass.default_action(self)

            image_data = morse.core.blenderapi.cameras()[self.name()].source

            # Convert the Z-Buffer
            self.local_data['points'] = self.converter.recover(image_data)
            self.capturing = True

            if (self._n > 0):
                self._n -= 1
                if (self._n == 0):
                    self.completed(status.SUCCESS)
        else:
            self.capturing = False


class DepthVideoCameraClass(VideoCameraClass):
    """
    This sensor generates a Depth 'image' from the camera perspective.

    "Depth images are published as sensor_msgs/Image encoded as 32-bit float.
    Each pixel is a depth (along the camera Z axis) in meters."
    [ROS Enhancement Proposal 118](http://ros.org/reps/rep-0118.html) on Depth
    Images.

    If you are looking for PointCloud data, you can use external tools like
    [depth_image_proc](http://ros.org/wiki/depth_image_proc) which will use the
    ``intrinsic_matrix`` and the ``image`` to generate it, or eventually the
    ``XYZCameraClass`` in this module.
    """

    _name = "Depth camera"

    add_data('image', 'none', 'buffer', "Z-Buffer captured by the camera, "
             "converted in meters. memoryview of float of size "
             "``(cam_width * cam_height * sizeof(float))`` bytes.")

    def __init__(self, obj, parent=None):
        """ Constructor method.

        Receives the reference to the Blender object.
        The second parameter should be the name of the object's parent.
        """
        # Call the constructor of the parent class
        VideoCameraClass.__init__(self, obj, parent)

        # Store the camera parameters necessary for image processing
        self.converter = ZBufferToDepth(self.near_clipping, self.far_clipping, \
                                        self.image_width, self.image_height)

    def default_action(self):
        """ Update the texture image. """
        # Grab an image from the texture
        if self.bge_object['capturing'] and (self._n != 0) :

            # Call the action of the parent class
            CameraClass.default_action(self)

            image_data = morse.core.blenderapi.cameras()[self.name()].source

            # Convert the Z-Buffer
            self.local_data['image'] = self.converter.recover(image_data)
            self.capturing = True

            if (self._n > 0):
                self._n -= 1
                if (self._n == 0):
                    self.completed(status.SUCCESS)
        else:
            self.capturing = False
