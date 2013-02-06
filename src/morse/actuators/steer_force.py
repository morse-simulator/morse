import logging; logger = logging.getLogger("morse." + __name__)
import morse.core.actuator
from morse.helpers.components import add_data

class SteerForce(morse.core.actuator.Actuator):
    """ 
    This actuator reads the values of the steering angle, the engine
    power and the braking force to drive a car like vehicle.  It is
    meant to work with robots implementing the `Blender Vehicle Wrapper
    <http://www.tutorialsforblender3d.com/Game_Engine/Vehicle/Vehicle_1.html>`_,
    such as the :doc:`Hummer robot <../robots/hummer>`.

    .. note:: 
        Robots implementing the Vehicle Wrapper must be pointing towards
        their local Y axis.  This means the robots will be oriented
        differently with respect to all other MORSE components.
    """

    _name = "Steer/Force Actuator"
    _short_desc = "Motion controller using engine force and steer angle speeds"

    add_data('steer', 0.0, "float",
                    'Angle of the wheels with respect to the vehicle \
                     (in radian)')
    add_data('force', 0.0, "float",
                      'The force applied to the traction wheels. \
                       A negative force will make the vehicle move forward. \
                       A positive force will make it go backwards.')
    add_data('brake', 0.0, "float",
                      'The force applied to the brake. \
                      It opposes to the force.')

    def __init__(self, obj, parent=None):
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__, self).__init__(obj, parent)

        logger.info('Component initialized')


    def default_action(self):
        """ Apply (steer, force) to the parent robot. """
        # Get the Blender object of the parent robot
        vehicle = self.robot_parent.vehicle

        # Update the steering value for these wheels:
        # The number at the end represents the wheel 'number' in the 
        # order they were created when initializing the robot.
        # Front wheels #0 and #1.
        # Rear wheels #2 and #3.
        vehicle.setSteeringValue(self.local_data['steer'], 0)
        vehicle.setSteeringValue(self.local_data['steer'], 1)

        # Update the Force (speed) for these wheels:
        vehicle.applyEngineForce(self.local_data['force'] * .4, 0)
        vehicle.applyEngineForce(self.local_data['force'] * .4, 1)
        vehicle.applyEngineForce(self.local_data['force'] * .4, 2)
        vehicle.applyEngineForce(self.local_data['force'] * .4, 3)

        # Brakes:
        # Applies the braking force to each wheel listed:
        # ['brakes'] = the game property value for the car labeled 'brakes'
        # Default value is 0:
        vehicle.applyBraking(self.local_data['brake'] * .1,  0)
        vehicle.applyBraking(self.local_data['brake'] * .1,  1)
        vehicle.applyBraking(self.local_data['brake'] * 1.3, 2)
        vehicle.applyBraking(self.local_data['brake'] * 1.3, 3)
