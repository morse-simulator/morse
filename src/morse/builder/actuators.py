import morse.builder.creator

class MotionController(morse.builder.creator.ActuatorCreator):
  def __init__(self, name="Motion_Controller"):
    morse.builder.creator.ActuatorCreator.__init__(self, name, 
      "morse/actuators/v_omega", "VWActuatorClass", "morse_vw_control")

