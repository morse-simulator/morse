import logging; logger = logging.getLogger("morse." + __name__)
from morse.middleware.ros import ROSPublisherTF
import rospy

class StaticTFPublisher(ROSPublisherTF):
    """ Publish the (static) transform between robot and this sensor/actuator with 1Hz. """
    last_ts = 0
    child_frame_id = None
    parent_frame_id = None

    def initialize(self):
        ROSPublisherTF.initialize(self)
        self.child_frame_id = self.kwargs.get('child_frame_id', self.frame_id)
        self.parent_frame_id = self.kwargs.get('parent_frame_id', 'base_link')
        logger.info("Initialized the ROS static TF publisher with frame_id '%s' " + \
                    "and child_frame_id '%s'", self.parent_frame_id, self.child_frame_id)

    def finalize(self):
        ROSPublisherTF.finalize(self)

    def default(self, ci='unused'):
        # publish with 1Hz
        if self.data['timestamp'] > self.last_ts + 1.0:
            # date timestamp forward, just like for static transform publisher
            ts = rospy.Time.from_sec(self.data['timestamp'] + 1.0)
            self.send_transform_robot(ts, self.child_frame_id, self.parent_frame_id)
            self.last_ts = self.data['timestamp']
