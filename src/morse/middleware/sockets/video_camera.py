import logging; logger = logging.getLogger("morse." + __name__)
import json
from morse.middleware.socket_mw import MorseSocketServ
from functools import partial

def init_extra_module(self, component_instance, function, mw_data):
    """ Setup the middleware connection with this data

    Prepare the middleware to handle the serialised data as necessary.
    """
    component_instance.output_functions.append(partial(MorseSocketServ.main_export, mw_data[-1], function))

def post_video_camera(self, component_instance):
    res = {}
    res['height'] = component_instance.image_height
    res['width'] = component_instance.image_width
    res['image'] = []

    image= bytes(component_instance.local_data['image'].image)
    for i in range(0, res['height'] * res['width'] * 4, 4):
        o = {}
        o ['r'] = image[i]
        o ['g'] = image[i+1]
        o ['b'] = image[i+2]
        o ['a'] = image[i+3]
        res['image'].append(o)

    return (json.dumps(res) + '\n').encode()
