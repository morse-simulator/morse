import logging; logger = logging.getLogger("morse." + __name__)
from morse.core.services import async_service
from morse.core import status
import bge
import mathutils
import morse.sensors.camera
import morse.sensors.zBuffTo3D

BLENDER_HORIZONTAL_APERTURE = 32.0

class DepthCameraClass(morse.sensors.camera.CameraClass):
    """ Video capture camera

    Generates a sequence of images viewed from the camera perspective.
    """

    def __init__(self, obj, parent=None):
        """ Constructor method.

        Receives the reference to the Blender object.
        The second parameter should be the name of the object's parent.
        """
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__, self).__init__(obj, parent)

        # Prepare the exportable data of this sensor
        self.local_data['3D_points'] = []

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
        morse.sensors.zBuffTo3D.init_image_parameters(alpha_u, alpha_u, self.near_clipping, self.far_clipping, self.image_width, self.image_height)

        # Variable to indicate this is a camera
        self.camera_tag = True

        logger.info('Component initialized')

    def __del__(self):
        """ Clean the memory allocated in the C module """
        morse.sensors.zBuffTo3D.release_memory()

    def interrupt(self):
        self._n = 0
        super(VideoCameraClass, self).interrupt()

    @async_service
    def capture(self, n):
        self._n = n

    def default_action(self):
        """ Update the texture image and compute a list of 3D points
        based on the zBuffer. """
        # Grab an image from the texture
        if self.blender_obj['capturing'] and (self._n != 0) :

            # Call the action of the parent class
            super(self.__class__, self).default_action()

            # NOTE: Blender returns the image as a binary string
            #  encoded as RGBA
            image_data = bge.logic.cameras[self.name()].source

            # Fill in the exportable data
            self.capturing = True

            #logger.debug ("Image: %s" % image_data.image)
            data_3d = morse.sensors.zBuffTo3D.recover_3d_point(image_data)
            self.local_data['3D_points'] = data_3d
            #logger.debug ("3D DATA: %s" % data_3d)

            if (self._n > 0):
                self._n -= 1
                if (self._n == 0):
                    self.completed(status.SUCCESS)
        else:
            self.capturing = False
