import logging; logger = logging.getLogger("morse." + __name__)
from morse.core.services import async_service
from morse.core import status, mathutils
import morse.core.blenderapi
import morse.sensors.camera
from morse.helpers.components import add_data
from morse.sensors.zbufferto3d import ZBufferTo3D

BLENDER_HORIZONTAL_APERTURE = 32.0

class DepthCameraClass(morse.sensors.camera.CameraClass):
    """
    This sensor generates a 3D point cloud from the camera perspective.
    """

    _name = "Depth camera"

    add_data('points', 'none', 'memoryview', "List of 3D points from the depth "
             "camera. memoryview of a set of float(x,y,z). The data is of size "
             "``(cam_width * cam_height * 12)`` bytes (12=3*sizeof(float).")
    add_data('intrinsic_matrix', 'none', 'mat3<float>',
        "The intrinsic calibration matrix, stored as a 3x3 row major Matrix.")

    def __init__(self, obj, parent=None):
        """ Constructor method.

        Receives the reference to the Blender object.
        The second parameter should be the name of the object's parent.
        """
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__, self).__init__(obj, parent)

        # Prepare the exportable data of this sensor
        self.local_data['points'] = []

        # Prepare the intrinsic matrix for this camera.
        # Note that the matrix is stored in row major
        intrinsic = mathutils.Matrix()
        intrinsic.identity()
        alpha_u = self.image_width  * \
                  self.image_focal / BLENDER_HORIZONTAL_APERTURE
        intrinsic[0][0] = alpha_u
        intrinsic[1][1] = alpha_u
        intrinsic[0][2] = self.image_width / 2.0
        intrinsic[1][2] = self.image_height / 2.0
        self.local_data['intrinsic_matrix'] = intrinsic

        self.capturing = False
        self._n = -1

        # Store the camera parameters necessary for image processing
        self.zbufferto3d = ZBufferTo3D(alpha_u, alpha_u, \
                                       self.near_clipping, self.far_clipping, \
                                       self.image_width, self.image_height)

        # Variable to indicate this is a camera
        self.camera_tag = True

        logger.info('Component initialized')

    def interrupt(self):
        self._n = 0
        super(DepthCameraClass, self).interrupt()

    @async_service
    def capture(self, n):
        """
        Capture **n** images

        :param n: the number of images to take. A negative number means
        take image indefinitely
        """
        self._n = n

    def default_action(self):
        """ Update the texture image and compute a list of 3D points
        based on the zBuffer. """
        # Grab an image from the texture
        if self.bge_object['capturing'] and (self._n != 0) :

            # Call the action of the parent class
            super(self.__class__, self).default_action()

            image_data = morse.core.blenderapi.cameras()[self.name()].source

            # Fill in the exportable data
            self.capturing = True

            self.local_data['points'] = self.zbufferto3d.recover(image_data)

            if (self._n > 0):
                self._n -= 1
                if (self._n == 0):
                    self.completed(status.SUCCESS)
        else:
            self.capturing = False
