from sensor_msgs.msg import Imu
from morse.middleware.ros import ROSPublisher

class DvlPublisher(ROSPublisher):
    """ Publish the data of the IMU sensor (without covariance). """
    ros_class = Imu
    default_frame_id = '/dvl'

    def default(self, ci='unused'):
        imu = Imu()
        imu.header = self.get_ros_header()

        #imu.orientation = 2#self.component_instance.bge_object.worldOrientation.to_quaternion()

        imu.angular_velocity.x = 2#self.data['angular_velocity'][0]
        imu.angular_velocity.y = 2#self.data['angular_velocity'][1]
        imu.angular_velocity.z = 2#self.data['angular_velocity'][2]

        imu.linear_acceleration.x = 2# self.data['linear_acceleration'][0]
        imu.linear_acceleration.y = 2# self.data['linear_acceleration'][1]
        imu.linear_acceleration.z = 2# self.data['linear_acceleration'][2]

        self.publish(imu)
