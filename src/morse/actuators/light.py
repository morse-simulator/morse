import logging; logger = logging.getLogger("morse." + __name__)
import bge
import morse.core.actuator

class LightActuatorClass(morse.core.actuator.MorseActuatorClass):
    """ Point light actuator

    http://www.blender.org/documentation/blender_python_api_2_57_release/bge.logic.html
    http://www.blender.org/documentation/blender_python_api_2_57_release/bge.types.html#bge.types.KX_LightObject
    http://www.blender.org/documentation/blender_python_api_2_57_release/bge.types.html#bge.types.KX_GameObject.children
    """

    def __init__(self, obj, parent=None):
        logger.info("%s initialization" % obj.name)
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(obj, parent)

        self.local_data['emit'] = obj['emit']

        logger.info('Component initialized')

    def default_action(self):
        """ Apply (v, w) to the parent robot. """
        # get the Blender Logic Controller
        co = bge.logic.getCurrentController()
        # get the Empty object parent of this Controller
        lightAct = co.owner
        # get the light which is a child of the Empty object
        light = lightAct.children[0]

        # switch on/off the light
        if self.local_data['emit']:
            light.energy = 1.0
        else:
            light.energy = 0.0

