import roslib; roslib.load_manifest('sensor_msgs');
from sensor_msgs.msg import Imu
from morse.middleware.ros import ROSPublisher

class ImuPublisher(ROSPublisher):

    def initalize(self):
        ROSPublisher.initalize(self, Imu)
        self.frame_id = self.kwargs.get("frame_id", "/imu")
        # get the IMU orientation to post in the ROS message
        self.orientation = self.component_instance.bge_object.worldOrientation.to_quaternion()

    def default(self, ci='unused'):
        """ Publish the data of the IMU sensor as a custom ROS Imu message """
        imu = Imu()
        imu.header = self.get_ros_header()

        imu.orientation = self.orientation

        imu.angular_velocity.x = self.data['angular_velocity'][0]
        imu.angular_velocity.y = self.data['angular_velocity'][1]
        imu.angular_velocity.z = self.data['angular_velocity'][2]

        imu.linear_acceleration.x = self.data['linear_acceleration'][0]
        imu.linear_acceleration.y = self.data['linear_acceleration'][1]
        imu.linear_acceleration.z = self.data['linear_acceleration'][2]

        self.publish(imu, component_instance)
