import morse.builder.creator
import morse.builder.blenderobjects

class MotionController(morse.builder.creator.ActuatorCreator):
    def __init__(self, name="Motion_Controller"):
        morse.builder.creator.ActuatorCreator.__init__(self, name, 
            "morse/actuators/v_omega", "VWActuatorClass", "morse_vw_control")

class Light(morse.builder.creator.ActuatorCreator):
    def __init__(self, name="LightAct"):
        morse.builder.creator.ActuatorCreator.__init__(self, name, 
            "morse/actuators/light", "LightActuatorClass", "morse_light")
        light = morse.builder.blenderobjects.Spot("LightSpot")
        self.append(light)
        self.properties(emit = True)

