import logging; logger = logging.getLogger("morse." + __name__)
import json
from morse.middleware.socket_datastream import SocketPublisher

class VideoPublisher(SocketPublisher):

    #    _type_name = "a JSON-encoded image: the height, the width and a list of height * width points {'r':..., 'g':..., 'b':..., 'a':...}"
    _type_name = "a JSON-encoded image"

    def encode(self):
        res = {}
        res['height'] = self.component_instance.image_height
        res['width'] = self.component_instance.image_width
        res['image'] = []

        image= bytes(self.component_instance.local_data['image'].image)
        for i in range(0, res['height'] * res['width'] * 4, 4):
            o = {}
            o ['r'] = image[i]
            o ['g'] = image[i+1]
            o ['b'] = image[i+2]
            o ['a'] = image[i+3]
            res['image'].append(o)

        return (json.dumps(res) + '\n').encode()
