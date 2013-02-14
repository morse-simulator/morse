import logging; logger = logging.getLogger("morse." + __name__)
import roslib; roslib.load_manifest('sensor_msgs'); roslib.load_manifest('rospy')
import rospy
from sensor_msgs.msg import Image, CameraInfo
from morse.middleware.ros import ROSPublisher

class VideoCameraPublisher(ROSPublisher):
    """ Publish the image from the Camera perspective.
    And send the intrinsic matrix information in a separate topic of type
    `sensor_msgs/CameraInfo <http://ros.org/wiki/rviz/DisplayTypes/Camera>`_.
    """
    ros_class = Image

    def initialize(self):
        self.kwargs['topic_suffix'] = '/image'
        ROSPublisher.initialize(self)
        # Generate a publisher for the CameraInfo
        self.topic_camera_info = rospy.Publisher(self.topic_name+'/camera_info', CameraInfo)

    def finalize(self):
        ROSPublisher.finalize(self)
        # Unregister the CameraInfo topic
        self.topic_camera_info.unregister()

    def default(self, ci='unused'):
        if not self.component_instance.capturing:
            return # press [Space] key to enable capturing

        image_local = self.data['image']

        image = Image()
        image.header = self.get_ros_header()
        image.header.frame_id += '/base_image'
        image.height = self.component_instance.image_height
        image.width = self.component_instance.image_width
        image.encoding = 'rgba8'
        image.step = image.width * 4

        # VideoTexture.ImageRender implements the buffer interface
        image.data = bytes(image_local)

        # fill this 3 parameters to get correcty image with stereo camera
        Tx = 0
        Ty = 0
        R = [1, 0, 0, 0, 1, 0, 0, 0, 1]

        intrinsic = self.data['intrinsic_matrix']

        camera_info = CameraInfo()
        camera_info.header = image.header
        camera_info.height = image.height
        camera_info.width = image.width
        camera_info.distortion_model = 'plumb_bob'
        camera_info.K = [intrinsic[0][0], intrinsic[0][1], intrinsic[0][2],
                         intrinsic[1][0], intrinsic[1][1], intrinsic[1][2],
                         intrinsic[2][0], intrinsic[2][1], intrinsic[2][2]]
        camera_info.R = R
        camera_info.P = [intrinsic[0][0], intrinsic[0][1], intrinsic[0][2], Tx,
                         intrinsic[1][0], intrinsic[1][1], intrinsic[1][2], Ty,
                         intrinsic[2][0], intrinsic[2][1], intrinsic[2][2], 0]

        self.publish(image)
        self.topic_camera_info.publish(camera_info)
