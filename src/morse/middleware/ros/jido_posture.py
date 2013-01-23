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

        js.name = [
           'kuka_arm_0_joint', 'kuka_arm_1_joint', 'kuka_arm_2_joint',
           'kuka_arm_3_joint', 'kuka_arm_4_joint', 'kuka_arm_5_joint',
           'kuka_arm_6_joint', 'head_pan_joint', 'head_tilt_joint'
        ]
        js.position = [
            self.data['seg0'],
            self.data['seg1'],
            self.data['seg2'],
            self.data['seg3'],
            self.data['seg4'],
            self.data['seg5'],
            self.data['seg6'],
            self.data['pan'],
            self.data['tilt']
        ]
        #js.velocity = [1, 1, 1, 1, 1, 1, 1]
        #js.effort = [50, 50, 50, 50, 50, 50, 50]

        self.publish(js)
