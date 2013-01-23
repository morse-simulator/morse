import roslib; roslib.load_manifest('sensor_msgs');
from sensor_msgs.msg import NavSatFix
from morse.middleware.ros import ROSPublisher

class NavSatFixPublisher(ROSPublisher):

    def initalize(self):
        ROSPublisher.initalize(self, NavSatFix)

    def default(self, ci='unused'):
        """ Publish the data of the gps sensor as a custom ROS NavSatFix message """
        gps = NavSatFix()
        gps.header = self.get_ros_header()

        # TODO ros.org/doc/api/sensor_msgs/html/msg/NavSatFix.html
        gps.latitude = self.data['x']
        gps.longitude = self.data['y']
        gps.altitude = self.data['z']

        self.publish(pose)
