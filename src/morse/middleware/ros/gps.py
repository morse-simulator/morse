from sensor_msgs.msg import NavSatFix
from morse.middleware.ros import ROSPublisher

class NavSatFixPublisher(ROSPublisher):
    """ Publish the GPS position of the robot. """
    ros_class = NavSatFix

    def default(self, ci='unused'):
        gps = NavSatFix()
        gps.header = self.get_ros_header()

        gps.latitude = self.data['x']
        gps.longitude = self.data['y']
        gps.altitude = self.data['z']

        self.publish(gps)
