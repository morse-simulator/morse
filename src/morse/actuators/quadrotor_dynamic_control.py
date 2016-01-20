import logging; logger = logging.getLogger("morse." + __name__)
import morse.core.actuator
from math import sqrt 
from morse.core.mathutils import Vector, Matrix
from morse.helpers.components import add_data, add_property

class QuadrotorDynamicControl(morse.core.actuator.Actuator):
    """
    This actuator reads speed of the four motors of the quadrotor, and
    computes the resulting force  / moment, following the dynamic model
    proposed in:
        - Backstepping and Sliding-mode Technique Applied to an Indoor
          Micro Quadrotor
        - Control of Complex Maneuvers for a Quadrotor UAV using
          Geometric Methods on SE(3)

    The actuator assumes that first and third properlers turns
    clockwise, while second and fourth turns counter-clockwise
    """
    
    _name = "Quadrotor dynamic controller"
    _short_desc = "Motion controller computing dynamic from propellers speed"
    
    add_data('engines', [0.0, 0.0, 0.0, 0.0], 'vec4<float>',
             'speed of each engines, in rad/s')
    add_property('_thrust_factor', 9.169e-06, 'ThrustFactor', 'float',
                 'thrust factor, in Ns²')
    add_property('_drag_factor', 2.4e-7, 'DragFactor', 'float',  'drag factor in Nms')
    add_property('_lever',  0.18, 'Lever', 'distance between center of \
            mass and propeller, in m')
    add_property('_configuration', '+', 'Configuration', 
                "A character between ['+', 'x'], '+' denoting a configuration "
                "where the drone X-axis follows the front axle, while 'x' "
                "denoting a configuration where the X-axis of the drone is between "
                "the two front arm of the drone")

    def __init__(self, obj, parent=None):
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        morse.core.actuator.Actuator.__init__(self, obj, parent)

        # a few references to ease matrix writing
        d = self._drag_factor
        b = self._thrust_factor
        l = self._lever
        bl = b * l
        bl_sq = bl * sqrt(2) / 2

        if self._configuration == '+':
            self.transformation = Matrix((
                    [b, b , b, b],
                    [0.0, -bl, 0.0, bl],
                    [bl, 0.0, -bl, 0.0],
                    [-d, d, -d, d]))
        elif self._configuration == 'x':
            self.transformation = Matrix((
                    [b, b , b, b],
                    [bl_sq, -bl_sq, -bl_sq, bl_sq],
                    [-bl_sq, -bl_sq, bl_sq, bl_sq],
                    [-d, d, -d, d]))
        else:
            logger.error("Invalid configuration %s" % self._configuration)

        logger.info("Component initialized, runs at %.2f Hz ", self.frequency)


    def default_action(self):
        """ Run attitude controller and apply resulting force and torque to the parent robot. """
        # Get the the parent robot
        robot = self.robot_parent

        engines_input = Vector(self.local_data['engines'])
        engines_input_sq = Vector.Fill(len(engines_input), 0.0)
        for i in range(0, len(engines_input)):
            engines_input_sq[i] = engines_input[i] * engines_input[i]
        moment_vector = self.transformation * engines_input_sq

        logger.debug("%s => %s" % (engines_input, moment_vector))

        force = (0.0, 0.0, moment_vector[0])
        torque = (moment_vector[1], moment_vector[2], moment_vector[3])

        # directly apply local forces and torques to the blender object of the parent robot
        robot.bge_object.applyForce(force, True)
        robot.bge_object.applyTorque(torque, True)
