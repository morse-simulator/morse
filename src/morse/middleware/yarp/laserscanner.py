import logging; logger = logging.getLogger("morse." + __name__)
from morse.middleware.yarp_datastream import YarpPublisher

class YarpLaserScannerPublisher(YarpPublisher):
    def encode(self, bottle):
        self.encode_message(bottle, self.data['point_list'],
                           self.component_name)

class YarpLaserScannerDistancePublisher(YarpPublisher):
    def encode(self, bottle):
        self.encode_message(bottle, self.data['range_list'],
                                   self.component_name)


