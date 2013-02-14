import logging; logger = logging.getLogger("morse." + __name__)
import roslib; roslib.load_manifest('sensor_msgs')
import struct
from sensor_msgs.msg import PointCloud2, PointField
from morse.middleware.ros import ROSPublisherTF

class XYZRGBPublisher(ROSPublisherTF):
    """ Publish the merged image and depth field from the Kinect perspective as XYZRGB point-cloud.
    And send the transformation between the camera and the robot through TF.
    """
    ros_class = PointCloud2

    def initialize(self):
        ROSPublisherTF.initialize(self)
        self.npixels = None

    def default(self, ci='unused'):
        if not self.component_instance.capturing:
            return # press [Space] key to enable capturing
        if not self.npixels:
            npx_video = self.component_instance.video_camera.image_width * \
                        self.component_instance.video_camera.image_height
            npx_depth = self.component_instance.depth_camera.image_width * \
                        self.component_instance.depth_camera.image_height
            assert(npx_depth == npx_video) # REQUIRED for merge
            self.npixels = npx_video

        video = self.data['video'] # RGBA bgl.Buffer!
        points = self.data['depth'] # memoryview XYZ

        pc2 = PointCloud2()
        pc2.header = self.get_ros_header()
        pc2.height = 1
        pc2.width = self.npixels
        pc2.is_dense = False
        pc2.is_bigendian = False
        pc2.fields = [PointField('x', 0, PointField.FLOAT32, 1),
                      PointField('y', 4, PointField.FLOAT32, 1),
                      PointField('z', 8, PointField.FLOAT32, 1),
                      PointField('r', 12, PointField.UINT8, 1),
                      PointField('g', 13, PointField.UINT8, 1),
                      PointField('b', 14, PointField.UINT8, 1)]
        pc2.point_step = 15 # XYZ * sizeof(float) + RGB
        pc2.row_step = self.npixels * 15

        pc2.data = bytes().join(merge_xyz_rgb(points, video, self.npixels))
        # bytes().join() because: object of type 'generator' has no len()
        # ROS serializer needs len()-compatible objects

        self.publish(pc2)
        self.send_transform_robot()

# TODO write this function in C
def merge_xyz_rgb(float32_xyz, uint8_rgba, npixels):
    uint8_rgba_mv = memoryview(uint8_rgba)
    px_xyz = 0
    # for i in range(width * height):
    for px_rgba in range(0, npixels * 4, 4):
        # take 12 bytes from float32_xyz + 3 (RGB) from uint8_rgba (drop alpha)
        px_xyz = px_rgba * 3
        yield bytes(float32_xyz[px_xyz:px_xyz+12])
        yield bytes(uint8_rgba_mv[px_rgba:px_rgba+3])
        #yield struct.pack('12B', *float32_xyz[px_xyz:px_xyz+12])
        #yield struct.pack('3B', *uint8_rgba_mv[px_rgba:px_rgba+3])
