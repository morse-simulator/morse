import roslib; roslib.load_manifest('sensor_msgs')
from sensor_msgs.msg import Range
from morse.middleware.ros import ROSPublisher

class RangePublisher(ROSPublisher):

    def initialize(self):
        ROSPublisher.initialize(self, Range)

    def default(self, ci='unused'):
        """ Publish the data of the infrared sensor as a ROS Range message """
        msg = Range()
        msg.radiation_type = Range.INFRARED
        msg.field_of_view = 20
        msg.min_range = 0
        msg.max_range = self.component_instance.bge_object['laser_range']
        tmp = msg.max_range
        for ray in self.data['range_list']:
            if tmp > ray:
                tmp = ray
        msg.range = tmp

        self.publish(msg)
