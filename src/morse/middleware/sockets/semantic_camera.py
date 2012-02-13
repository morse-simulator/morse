import logging; logger = logging.getLogger("morse." + __name__)
import json
from morse.middleware.socket_mw import MorseSocketServ
from functools import partial

def init_extra_module(self, component_instance, function, mw_data):
    """ Setup the middleware connection with this data

    Prepare the middleware to handle the serialised data as necessary.
    """
    component_instance.output_functions.append(partial(MorseSocketServ.main_export, mw_data[-1], function))

def post_semantic_camera(self, component_instance):
    data = []
    for obj in component_instance.local_data['visible_objects']:
        o = {}
        o['name'] = obj['name']
        o['type'] = obj['type']
        pos = obj['position']
        o['position'] = { 'x' : pos.x, 'y': pos.y, 'z' : pos.z }
        orientation = obj['orientation']
        o['orientation'] = { 'w' : orientation.w, 'x' : orientation.x, 'y' : orientation.y, 
                             'z' : orientation.z }
        data.append(o)

    return (json.dumps(data) + '\n').encode()
