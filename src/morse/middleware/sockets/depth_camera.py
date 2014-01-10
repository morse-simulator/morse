#import struct
import base64
import json
#import binascii
from collections import OrderedDict
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

        points = memoryview(self.component_instance.local_data['points'])
        p = bytes(points)
        data = base64.b64encode(p)
        """
        removes leading "b' "
        """
        data = str(data,'ascii')	
        intrinsic = self.component_instance.local_data['intrinsic_matrix']

        intrinisc_matrix = []
        for i in range(0, 3):
            for j in range (0, 3):
                intrinisc_matrix.append(intrinsic[i][j])		
		
        res = OrderedDict ( [('timestamp', self.data['timestamp']) ,
		        ('height', self.component_instance.image_height),
		        ('width', self.component_instance.image_width),
		        ('points', data),
		        ('intrinsic_matrix', intrinisc_matrix)
		        ])
        return (json.dumps(res) + '\n').encode()
