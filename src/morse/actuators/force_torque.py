import logging; logger = logging.getLogger("morse." + __name__)
import bge
import morse.core.actuator

class ForceTorqueActuatorClass(morse.core.actuator.MorseActuatorClass):
    """ Motion controller using force and torque

    This class will read force and torque as input 
    from an external middleware, and then apply them
    to the parent robot.
    """

    def __init__(self, obj, parent=None):
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(obj, parent)

        self.local_data['force'] = [0.0, 0.0, 0.0]
        self.local_data['torque'] = [0.0, 0.0, 0.0]
        
        logger.info('Component initialized')


    def default_action(self):
        """ Apply (force, torque) to the parent robot. """
        # Get the Blender object of the parent robot
        parent = self.robot_parent
        
        # apply local forces and torques to the blender object of the parent robot
        parent.blender_obj.applyForce(self.local_data['force'], True)
        parent.blender_obj.applyTorque(self.local_data['torque'], True)
