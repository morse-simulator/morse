import morse.builder.creator
import morse.builder.blenderobjects

class ATRV(morse.builder.creator.RobotCreator):
    def __init__(self, name="LightAct"):
        morse.builder.creator.RobotCreator.__init__(self, name, 
            "morse/robots/atrv", "ATRVClass", "atrv")
        self.append_collada()

