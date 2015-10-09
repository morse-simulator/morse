import logging; logger = logging.getLogger("morse." + __name__)
import morse.core.actuator
from morse.helpers.components import add_data, add_property
from morse.core import mathutils
from morse.helpers.morse_math import normalise_angle

class Orientation(morse.core.actuator.Actuator):
    """
    Motion controller changing the robot orientation.

    This actuator reads the values of angles of rotation around the 3
    axis and applies them to the associated robot. This rotation is
    applied with the speed provided in properties. For backward compatibility
    reasons, the default speed is infinite, so the rotation is instantaneous.
    Angles are expected in radians.
    """

    _name = "Orientation Actuator"
    _short_desc = "An actuator to change robot orientation."

    add_data('yaw', 'Initial robot yaw', "float",
                    'Rotation of the robot around Z axis, in radian')
    add_data('pitch', 'Initial robot pitch', "float",
                      'Rotation of the robot around Y axis, in radian')
    add_data('roll', 'Initial robot roll', "float",
                      'Rotation of the robot around X axis, in radian')

    add_property('_speed', float('inf'), 'speed', 'float',
                 'Rotation speed, in radian by sec')
    add_property('_tolerance', 0.02, 'tolerance', 'float',
                 'Tolerance in radian to decide if the robot has reached the goal')
    add_property('_type', 'Velocity', 'ControlType', 'string',
                 "Kind of control, can be one of ['Velocity', 'Position']")

    def __init__(self, obj, parent=None):
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        morse.core.actuator.Actuator.__init__(self, obj, parent)

        self.orientation = self.bge_object.orientation.to_euler('XYZ')

        self.local_data['yaw'] = self.orientation.z
        self.local_data['pitch'] = self.orientation.y
        self.local_data['roll'] = self.orientation.x

        logger.info('Component initialized')

    def default_action(self):
        """ Change the parent robot orientation. """

        if self._speed == float('inf'):
            # New parent orientation
            orientation = mathutils.Euler([self.local_data['roll'],
                                           self.local_data['pitch'],
                                           self.local_data['yaw']])

            self.robot_parent.force_pose(None, orientation.to_matrix())
        else:
            goal = [self.local_data['roll'], self.local_data['pitch'], self.local_data['yaw']]
            current_rot = [self.position_3d.roll, self.position_3d.pitch, self.position_3d.yaw]
            cmd = [0.0, 0.0, 0.0]
            for i in range(0, 3):
                diff = goal[i] - current_rot[i]
                diff = normalise_angle(diff)
                diff_abs = abs(diff)
                if diff_abs < self._tolerance:
                    cmd[i] = 0.0
                else:
                    sign = diff_abs / diff
                    if diff_abs > self._speed / self._frequency:
                        cmd[i] = sign * self._speed
                    else:
                        cmd[i] = diff_abs * self._frequency
                if self._type == 'Position':
                    cmd[i] /= self._frequency 

            self.robot_parent.apply_speed(self._type, [0.0, 0.0, 0.0], cmd)
