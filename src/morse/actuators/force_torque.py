import logging; logger = logging.getLogger("morse." + __name__)
from morse.core import mathutils
import morse.core.actuator
from morse.helpers.components import add_data, add_property

class ForceTorque(morse.core.actuator.Actuator):
    """
    This class will read force and torque as input 
    from an external middleware.
    The forces and torques are transformed from the actuator frame to the
    parent robot frame and then applied to the robot blender object.
    If the property RobotFrame is set to True it will be applied
    directly in the robot frame without changes.
    """

    _name = "Force/Torque Motion Controller"
    _short_desc="Motion controller using force and torque"


    add_data('force', [0.0, 0.0, 0.0], "vec3<float>", "force along x, y, z")
    add_data('torque', [0.0, 0.0, 0.0], "vec3<float>", "torque around x, y, z")

    add_property('_robot_frame', False, 'RobotFrame', 'bool', 'If set to true '
            'the inputs are applied in the Robot coordinate frame instead of the '
            'actuator frame.')


    def __init__(self, obj, parent=None):
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        morse.core.actuator.Actuator.__init__(self, obj, parent)

        logger.info('Component initialized')


    def default_action(self):
        """ Apply (force, torque) to the parent robot. """
        # Get the the parent robot
        robot = self.robot_parent

        if self._robot_frame:
            # directly apply local forces and torques to the blender object of the parent robot
            robot.bge_object.applyForce(self.local_data['force'], True)
            robot.bge_object.applyTorque(self.local_data['torque'], True)
        else:
            (loc, rot, scale) = robot.position_3d.transformation3d_with(self.position_3d).matrix.decompose()
            # rotate into robot frame, but still at actuator origin
            force = rot * mathutils.Vector(self.local_data['force'])
            torque = rot * mathutils.Vector(self.local_data['torque'])
            # add torque due to lever arm
            torque += loc.cross(force)
            robot.bge_object.applyForce(force, True)
            robot.bge_object.applyTorque(torque, True)
