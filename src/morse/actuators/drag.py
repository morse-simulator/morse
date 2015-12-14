import logging; logger = logging.getLogger("morse." + __name__)
import morse.core.actuator
from morse.core.mathutils import Vector
from morse.helpers.components import add_property
from math import copysign

class Drag(morse.core.actuator.Actuator):
    """
    This actuator allows to simulate the drag force, or fluid
    resistance. It is not controlable, as it depends only of robot
    parameters, and environment.

    The drag force is generally written as:

    Fd = 0.5 * rho * vel_sq * A * Cd

    where
        - rho is the density of the fluid (env dependant)
        - A is the cross sectional area (robot dependent)
        - Cd  is the drag coefficient,  related to the object shape and
          the Reynolds Number

    All these values are aggregated into the property DragCoeff, to ease
    the configuration of the component.
    """
    
    _name = "Drag"
    _short_desc = "Actuator allowing to simulate the drag force"
    

    add_property('_drag_coeff_x', 0.09, 'DragCoeffX', 'float'
                'An aggregated value to scale the drag force on X robot axis')
    add_property('_drag_coeff_y', 0.09, 'DragCoeffY', 'float'
                'An aggregated value to scale the drag force on Y robot axis')
    add_property('_drag_coeff_z', 0.5, 'DragCoeffZ', 'float'
                'An aggregated value to scale the drag force on Z robot axis')

    def __init__(self, obj, parent=None):
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        morse.core.actuator.Actuator.__init__(self, obj, parent)

        # Make new reference to the robot velocities (mathutils.Vector)
        self.robot_vel = self.robot_parent.bge_object.localLinearVelocity

        self._drag_coeff = [self._drag_coeff_x,
                            self._drag_coeff_y,
                            self._drag_coeff_z]
        logger.info("Component initialized, runs at %.2f Hz ", self.frequency)


    def default_action(self):
        robot = self.robot_parent
        drag = Vector((0.0, 0.0, 0.0))
        for i in range(3):
            v = self.robot_vel[i]
            drag[i] = - copysign(1.0, v) * v  * v * self._drag_coeff[i]

        logger.debug("Drag force %s" % drag)
        robot.bge_object.applyForce(drag, True)
