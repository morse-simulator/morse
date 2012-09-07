import logging; logger = logging.getLogger("morse." + __name__)
from morse.core.services import async_service
from morse.core import status
import bge
import mathutils
import morse.core.blenderapi
import morse.sensors.camera

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
        self.alpha_u = self.image_width  * \
                       self.image_focal / BLENDER_HORIZONTAL_APERTURE
        intrinsic[0][0] = self.alpha_u
        intrinsic[1][1] = self.alpha_u
        intrinsic[0][2] = self.image_width / 2.0
        intrinsic[1][2] = self.image_height / 2.0
        self.local_data['intrinsic_matrix'] = intrinsic

        self.capturing = False
        self._n = -1

        # Variable to indicate this is a camera
        self.camera_tag = True

        logger.info('Component initialized')

        self.u_0 = self.image_width / 2
        self.v_0 = self.image_height / 2
        self.alpha_v = self.alpha_u

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

            # NOTE: Blender returns the image as a binary string encoded as RGBA
            image_data = morse.core.blenderapi.cameras()[self.name()].source
            # get a 1D depth bgl.Buffer of float from 0.0 to 1.0
            depth = bge.texture.imageToArray(image_data, 'F')

            # Fill in the exportable data
            self.capturing = True

            data_3d = self.recover_3d_point(depth)
            self.local_data['3D_points'] = data_3d

            if (self._n > 0):
                self._n -= 1
                if (self._n == 0):
                    self.completed(status.SUCCESS)
        else:
            self.capturing = False

    def recover_3d_point(self, depth):
        """ Convert a 1D depth bgl.Buffer into an array of 3D points
        """
        point_list = []
        for pixel, z_f in enumerate(depth):
            z_n = 2.0 * z_f - 1.0
            z_e = 2.0 * self.near_clipping * self.far_clipping / \
                 (self.far_clipping + self.near_clipping - z_n * \
                 (self.far_clipping - self.near_clipping))
            # The image we receive is stored as a single array of floats.
            # Pixel 0, 0 in the data is located at the bottom left, according to
            # the OpenGL conventions.
            # We need to convert this frame of reference to (u, v), starting at
            # the top left
            u = pixel % self.image_width
            v = self.image_height - (pixel / self.image_width)
            # Use the intrinsic matrix of the camera view to get the 3D
            # coordinates corresponding to each pixel, with respect to the camera
            x = z_e * (u - self.u_0) / self.alpha_u
            y = z_e * (v - self.v_0) / self.alpha_v
            # Store the x, y, z coordinates in the list
            point_list.append([x, y, z_e])
        return point_list
