import logging; logger = logging.getLogger("morse." + __name__)
from sensor_msgs.msg import PointCloud2, PointField
from morse.middleware.ros import ROSPublisherTF

class DepthCameraPublisher(ROSPublisherTF):
    """ Publish the depth field from the Camera perspective as XYZ point-cloud.
    And send the transformation between the camera and the robot through TF.
    """
    ros_class = PointCloud2

    def default(self, ci='unused'):
        if not self.component_instance.capturing:
            return # press [Space] key to enable capturing

        # This message holds a collection of N-dimensional points, which may
        # contain additional information such as normals, intensity, etc. The
        # point data is stored as a binary blob, its layout described by the
        # contents of the "fields" array.

        # The point cloud data may be organized 2d (image-like) or 1d
        # (unordered). Point clouds organized as 2d images may be produced by
        # camera depth sensors such as stereo or time-of-flight.
        pc2 = PointCloud2()
        # Time of sensor data acquisition, and the coordinate frame ID (for 3d
        # points).
        pc2.header = self.get_ros_header()
        # 2D structure of the point cloud. If the cloud is unordered, height is
        # 1 and width is the length of the point cloud.
        pc2.height = 1
        pc2.width = self.data['nb_points']
        # Describes the channels and their layout in the binary data blob.
        pc2.fields = [PointField('x', 0, PointField.FLOAT32, 1),
                      PointField('y', 4, PointField.FLOAT32, 1),
                      PointField('z', 8, PointField.FLOAT32, 1)]
        pc2.is_dense = True         # True if there are no invalid points
        pc2.is_bigendian = False    # Is this data bigendian?
        pc2.point_step = 12         # Length of a point in bytes
        pc2.row_step = len(self.data['points']) # Length of a row in bytes

        # Actual point data, size is (row_step*height)
        # memoryview from PyMemoryView_FromMemory() implements the buffer interface
        pc2.data = bytes(self.data['points'])

        self.publish_with_robot_transform(pc2)

