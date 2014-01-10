import sys
import json
import base64
import logging; logger = logging.getLogger("morse." + __name__)
from morse.middleware.socket_datastream import SocketPublisher

class DepthCameraPublisher(SocketPublisher):
    """
    Data of the DepthCamera gets JSON encoded to send via sockets
    """

    _type_name = 'a JSON-Encoded message for the DepthCamera'

    def encode(self):
        if not self.component_instance.capturing:
            return bytes() # press [Space] key to enable capturing

        points = self.data['points']

        if sys.version_info < (3,4):
            points = bytes( points )

        data = base64.b64encode( points ).decode() # get string
        intrinsic = [ list(vec) for vec in self.data['intrinsic_matrix'] ]

        res = {
            'timestamp': self.data['timestamp'],
            'height':    self.component_instance.image_height,
            'width':     self.component_instance.image_width,
            'points':    data,
            'intrinsic_matrix': intrinsic,
        }

        return (json.dumps(res) + '\n').encode()
