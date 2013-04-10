from morse.builder.creator import ActuatorCreator

class @classname@(ActuatorCreator):
    def __init__(self, name=None):
        ActuatorCreator.__init__(self, name, \
                                 "@env@.actuators.@name@.@classname@",\
                                 "@name@")

