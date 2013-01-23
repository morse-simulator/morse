import roslib; roslib.load_manifest('sensor_msgs')
from sensor_msgs.msg import JointState
from morse.middleware.ros import ROSPublisher

class JointStatePublisher(ROSPublisher):

    def initalize(self):
        ROSPublisher.initalize(self, JointState)

    def default(self, ci='unused'):
        """ Publish the data of the posture sensor as a ROS JointState message """
        js = JointState()
        js.header = self.get_ros_header()
        js.name = ['head_pan_joint', 'head_tilt_joint']

        js.position = [
            self.data['pan'],
            self.data['tilt']
        ]

        self.publish(js)
