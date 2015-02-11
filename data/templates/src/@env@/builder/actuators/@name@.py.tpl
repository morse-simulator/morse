from morse.builder.creator import ActuatorCreator

class @classname@(ActuatorCreator):
    _classpath = "@env@.actuators.@name@.@classname@"
    _blendname = "@name@"

    def __init__(self, name=None):
        ActuatorCreator.__init__(self, name)

