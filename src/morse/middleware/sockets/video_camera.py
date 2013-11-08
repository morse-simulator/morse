import logging; logger = logging.getLogger("morse." + __name__)
import json
from morse.middleware.socket_datastream import SocketPublisher

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
