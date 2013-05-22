import logging; logger = logging.getLogger("morse." + __name__)
import roslib; roslib.load_manifest('sensor_msgs')
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

        points = self.data['points']
        size = len(points)

        pc2 = PointCloud2()
        pc2.header = self.get_ros_header()
        pc2.height = 1
        pc2.width = int(size / 12)
        pc2.is_dense = False
        pc2.is_bigendian = False
        pc2.fields = [PointField('x', 0, PointField.FLOAT32, 1),
                      PointField('y', 4, PointField.FLOAT32, 1),
                      PointField('z', 8, PointField.FLOAT32, 1)]
        pc2.point_step = 12
        pc2.row_step = size

        # memoryview from PyMemoryView_FromMemory() implements the buffer interface
        pc2.data = bytes(points)

        self.publish(pc2)
        self.send_transform_robot()
