from morse.builder.creator import SensorCreator

class @classname@(SensorCreator):
    def __init__(self, name=None):
        SensorCreator.__init__(self, name, \
                               "@env@.sensors.@name@.@classname@",\
                               "@name@")

