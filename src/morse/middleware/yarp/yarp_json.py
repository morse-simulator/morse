import logging; logger = logging.getLogger("morse." + __name__)
import json
from collections import OrderedDict
from morse.middleware.yarp_datastream import YarpPublisher, YarpReader
from morse.middleware.socket_datastream import MorseEncoder


class YarpJsonPublisher(YarpPublisher):
    _type_name = "json encoded data in yarp::bottle"
    def encode(self, bottle):
        js = json.dumps(self.data, cls=MorseEncoder)
        bottle.addString(js)

class YarpJsonReader(YarpReader):
    def default(self, ci):
        message_bottle = self.port.read(False)

        if message_bottle != None:
            message_data = message_bottle.get(0).toString()

            # Deserialise the data directly into a temporary ordered dictionary
            json_dict = json.loads(message_data, object_pairs_hook=OrderedDict)
            # Fill the component's 'local_data' dictionary,
            #  while also casting to the correct data types
            for variable, data in self.data.items():
                if isinstance(data, int):
                    self.data[variable] = int(json_dict[variable])
                elif isinstance(data, float):
                    self.data[variable] = float(json_dict[variable])
                elif isinstance(data, str):
                    self.data[variable] = json_dict[variable]
                else:
                    logger.error("Unknown data type at 'read_json_data'")

class YarpJsonWaypointReader(YarpReader):

    def default(self, ci):
        """
        Read a destination waypoint in the format used for ROSACE

        The structure read by JSON is expected to have two properties:
        point: structure with 'x', 'y', 'z'
        radius: the tolerance around those points
        """
        message_bottle = self.port.read(False)
        if message_bottle != None:
            message_data = message_bottle.get(0).toString()

            # Deserialise the data directly into a temporary ordered dictionary
            json_dict = json.loads(message_data, object_pairs_hook=OrderedDict)
            point = json_dict['point']
            tolerance = json_dict['radius']
            self.data['x'] = float(point['x'])
            self.data['y'] = float(point['y'])
            self.data['z'] = float(point['z'])
            self.data['tolerance'] = float(tolerance)
            self.data['speed'] = 3.0

