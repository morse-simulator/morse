import logging; logger = logging.getLogger("morse." + __name__)
from morse.core.services import async_service
from morse.core import status
import morse.core.blenderapi
from morse.core import mathutils
import morse.sensors.camera
from morse.helpers.components import add_data
import copy

BLENDER_HORIZONTAL_APERTURE = 32.0

class VideoCamera(morse.sensors.camera.Camera):
    """
    This sensor emulates a single video camera. It generates a series of
    RGBA images.  Images are encoded as binary char arrays, with 4 bytes
    per pixel.

    The cameras make use of Blender's **bge.texture** module, which
    requires a graphic card capable of GLSL shading.  Also, the 3D view
    window in Blender must be set to draw **Textured** objects.

    Camera calibration matrix
    -------------------------

    The camera configuration parameters implicitly define a geometric camera in
    blender units. Knowing that the **cam_focal** attribute is a value that
    represents the distance in Blender unit at which the largest image dimension is
    32.0 Blender units, the camera intrinsic calibration matrix is defined as

      +--------------+-------------+---------+
      | **alpha_u**  |      0      | **u_0** |
      +--------------+-------------+---------+
      |       0      | **alpha_v** | **v_0** |
      +--------------+-------------+---------+
      |       0      |      0      |    1    |
      +--------------+-------------+---------+

    where:

    - **alpha_u** == **alpha_v** = **cam_width** . **cam_focal** / 32 (we suppose
      here that **cam_width** > **cam_height**. If not, then use **cam_height** in
      the formula)
    - **u_0** = **cam_height** / 2
    - **v_0** = **cam_width** / 2
    """

    _name = "Video camera"
    _short_desc = "A camera capturing RGBA image"

    add_data('image', 'none', 'buffer',
           "The data captured by the camera, stored as a Python Buffer \
            class  object. The data is of size ``(cam_width * cam_height * 4)``\
            bytes. The image is stored as RGBA.")
    add_data('intrinsic_matrix', 'none', 'mat3<float>',
        "The intrinsic calibration matrix, stored as a 3x3 row major Matrix.")

    def __init__(self, obj, parent=None):
        """ Constructor method.

        Receives the reference to the Blender object.
        The second parameter should be the name of the object's parent.
        """
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(VideoCamera, self).__init__(obj, parent)

        # Prepare the exportable data of this sensor
        self.local_data['image'] = ''

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

        # Variable to indicate this is a camera
        self.camera_tag = True

        # Position of the robot where the last shot is taken
        self.robot_pose = copy.copy(self.robot_parent.position_3d)

        logger.info("Component initialized, runs at %.2f Hz ", self.frequency)

    def interrupt(self):
        self._n = 0
        super(VideoCamera, self).interrupt()

    @async_service
    def capture(self, n):
        """
        Capture **n** images

        :param n: the number of images to take. A negative number means
                  take image indefinitely
        """
        self._n = n

    def default_action(self):
        """ Update the texture image. """

        # Grab an image from the texture
        if self.bge_object['capturing'] and (self._n != 0) :

            # Call the action of the parent class
            super(self.__class__, self).default_action()

            # NOTE: Blender returns the image as a binary string
            #  encoded as RGBA
            image_data = morse.core.blenderapi.cameras()[self.name()].source

            self.robot_pose = copy.copy(self.robot_parent.position_3d)

            # Fill in the exportable data
            self.local_data['image'] = image_data
            self.capturing = True

            if (self._n > 0):
                self._n -= 1
                if (self._n == 0):
                    self.completed(status.SUCCESS)
        else:
            self.capturing = False
