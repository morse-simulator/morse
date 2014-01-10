import sys
import json
import base64
import logging; logger = logging.getLogger("morse." + __name__)
from morse.middleware.socket_datastream import SocketPublisher

class VideoCameraPublisher(SocketPublisher):
    """ Publish a base64 encoded image """

    _type_name = 'base64 encoded image'

    def encode(self):
        if not self.component_instance.capturing:
            return bytes() # press [Space] key to enable capturing

        image = self.data['image']

        if sys.version_info < (3,4):
            image = bytes( image )
        # Python 3.4 base64.b64encode supports memoryview, which is faster
        # TODO Blender 2.7 ? else: image = memoryview( image )

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

#
# deprecated publisher
#
class VideoPublisher(SocketPublisher):

    #    _type_name = "a JSON-encoded image: the height, the width and a list of height * width points {'r':..., 'g':..., 'b':..., 'a':...}"
    _type_name = "a JSON-encoded image"

    def encode(self):
        if not self.component_instance.capturing:
            return bytes() # press [Space] key to enable capturing

        res = {'height': self.component_instance.image_height,
               'width': self.component_instance.image_width,
               'image': []}

        # bge.texture.ImageRender is an image buffer, which is
        #   faster to convert than its raw `image` attribute.
        image = memoryview(self.component_instance.local_data['image'])
        for i in range(0, res['height'] * res['width'] * 4, 4):
            o = {'r': image[i],
                 'g': image[i + 1],
                 'b': image[i + 2],
                 'a': image[i + 3]}
            res['image'].append(o)

        return (json.dumps(res) + '\n').encode()
