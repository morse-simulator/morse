import logging; logger = logging.getLogger("morse." + __name__)
import json
import rospy
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from morse.middleware.ros import ROSPublisherTF
from morse.middleware.socket_datastream import MorseEncoder
from morse.middleware.ros.tfMessage import tfMessage

class SemanticCameraPublisher(ROSPublisherTF):
    """ Publish the data of the semantic camera as JSON in a ROS String message.
    And send TF transform between '/map' and ``object.name``.
    """
    ros_class = String

    def initialize(self):
        if not self.component_instance.relative:
            self.default_frame_id = '/map'
        self.pub_tf = self.kwargs.get('pub_tf', True)
        self.pub_camera_info = self.kwargs.get('pub_camera_info', True)
        ROSPublisherTF.initialize(self)
        if self.pub_camera_info:
            self.topic_camera_info = rospy.Publisher(self.topic_name+'/camera_info', CameraInfo,
                                                     queue_size=self.determine_queue_size())

    def default(self, ci='unused'):
        tfs = []
        for obj in self.data['visible_objects']:
            tfs.append(self.createTransform(obj['position'], obj['orientation'],
                                            self.get_time(), str(obj['name']), self.frame_id))
            # send tf-frame for every object
            # self.sendTransform(obj['position'], obj['orientation'],
            #                    self.get_time(), str(obj['name']), self.frame_id)
        if tfs:
            self.publish_tf(tfMessage(tfs))
        if self.pub_tf:
            self.send_transform_robot()
        if self.pub_camera_info:
            # Implementation copied from video camera
            intrinsic = self.data['intrinsic_matrix']
            Tx = 0
            Ty = 0
            R = [1, 0, 0, 0, 1, 0, 0, 0, 1]
            camera_info = CameraInfo()
            camera_info.header = self.get_ros_header()
            camera_info.height = int(2 * intrinsic[1][2])
            camera_info.width = int(2 * intrinsic[0][2])
            camera_info.distortion_model = 'plumb_bob'
            camera_info.D = []
            camera_info.K = [intrinsic[0][0], intrinsic[0][1], intrinsic[0][2],
                             intrinsic[1][0], intrinsic[1][1], intrinsic[1][2],
                             intrinsic[2][0], intrinsic[2][1], intrinsic[2][2]]
            camera_info.R = R
            camera_info.P = [intrinsic[0][0], intrinsic[0][1], intrinsic[0][2], Tx,
                             intrinsic[1][0], intrinsic[1][1], intrinsic[1][2], Ty,
                             intrinsic[2][0], intrinsic[2][1], intrinsic[2][2], 0]
            self.topic_camera_info.publish(camera_info)
        string = String()
        string.data = json.dumps(self.data['visible_objects'], cls=MorseEncoder)
        self.publish(string)
        

class SemanticCameraPublisherLisp(ROSPublisherTF):
    """ Publish the data of the semantic camera as a ROS String message,
    that contains a lisp-list (each field are separated by a space).

    This function was designed for the use with CRAM and the Adapto group.
    """
    ros_class = String

    def initialize(self):
        if not self.component_instance.relative:
            self.default_frame_id = '/map'
        ROSPublisherTF.initialize(self)

    def default(self, ci='unused'):
        string = String()
        string.data = "("
        for obj in self.data['visible_objects']:
            description = obj['description'] or '-'

            # send tf-frame for every object
            self.sendTransform(obj['position'], obj['orientation'],
                               self.get_time(), str(obj['name']), self.frame_id)

            # Build string from name, description, location and orientation in the global world frame
            string.data += "(" + str(obj['name']) + " " + description + " " + \
                           str(obj['position'].x) + " " + \
                           str(obj['position'].y) + " " + \
                           str(obj['position'].z) + " " + \
                           str(obj['orientation'].x) + " " + \
                           str(obj['orientation'].y) + " " + \
                           str(obj['orientation'].z) + " " + \
                           str(obj['orientation'].w) + ")"

        string.data += ")"
        self.publish(string)

class TeleportingSemanticCameraPublisher(SemanticCameraPublisher):
    def default(self, ci='unused'):
        if not self.data['objects_queue'].empty():
            self.data['visible_objects'] = self.data['objects_queue'].get()
            SemanticCameraPublisher.default(self, ci)
