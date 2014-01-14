import sys
import json
import base64
import logging; logger = logging.getLogger("morse." + __name__)
from morse.middleware.socket_datastream import SocketPublisher

class VideoCameraPublisher(SocketPublisher):
    """ Publish a base64 encoded RGBA image """

    _type_name = 'base64 encoded RGBA image'

    def process(self, image):
        if sys.version_info < (3,4):
            return bytes( image )
        else:
            # Python 3.4 base64.b64encode supports memoryview, which is faster
            # TODO Blender 2.7 ? else: image = memoryview( image )
            return image

    def encode(self):
        if not self.component_instance.capturing:
            return bytes() # press [Space] key to enable capturing

        image = self.process( self.data['image'] )

        data = base64.b64encode( image ).decode() # get string
        intrinsic = [ list(vec) for vec in self.data['intrinsic_matrix'] ]

        res = {
            'timestamp': self.data['timestamp'],
            'height':    self.component_instance.image_height,
            'width':     self.component_instance.image_width,
            'image':     data,
            'intrinsic_matrix': intrinsic,
        }

        return (json.dumps(res) + '\n').encode()

class Video8uPublisher(VideoCameraPublisher):
    """ Publish a base64 encoded grayscale (8U) image """

    _type_name = 'base64 encoded grayscale (8U) image'

    def process(self, image):
        memv = memoryview( image )
        # Grayscale model used for HDTV developed by the ATSC (Wikipedia)
        i8u = [ int(0.2126 * memv[index] +
                    0.7152 * memv[index + 1] +
                    0.0722 * memv[index + 2] )
                for index in range(0, len(memv), 4) ]
        return bytearray( i8u )
