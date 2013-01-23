import logging; logger = logging.getLogger("morse." + __name__)
import roslib; roslib.load_manifest('sensor_msgs')
from sensor_msgs.msg import PointCloud2, PointField
from morse.middleware.ros import ROSPublisher

class DepthCameraPublisher(ROSPublisher):

    def initalize(self):
        ROSPublisher.initalize(self, PointCloud2)

    def default(self, ci='unused'):
        """ Publish the data of the Camera as a ROS PointCloud2 message """
        if not self.component_instance.capturing:
            return # press [Space] key to enable capturing

        points = self.data['points']
        width = self.component_instance.image_width * self.component_instance.image_height
        size = len(points)

        pc2 = PointCloud2()
        pc2.header = self.get_ros_header()
        pc2.height = 1
        pc2.width = width
        pc2.is_dense = False
        pc2.is_bigendian = False
        pc2.fields = [PointField('x', 0, PointField.FLOAT32, 1),
                      PointField('y', 4, PointField.FLOAT32, 1),
                      PointField('z', 8, PointField.FLOAT32, 1)]
        pc2.point_step = int(size / width)
        pc2.row_step = size

        # memoryview from PyMemoryView_FromMemory() implements the buffer interface
        pc2.data = bytes(points)

        self.publish(pc2)
