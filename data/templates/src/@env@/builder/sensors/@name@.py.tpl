from morse.builder.creator import SensorCreator

class @classname@(SensorCreator):
    _classpath = "@env@.sensors.@name@.@classname@"
    _blendname = "@name@"

    def __init__(self, name=None):
        SensorCreator.__init__(self, name)

