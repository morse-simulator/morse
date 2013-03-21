import logging; logger = logging.getLogger("morse." + __name__)
from morse.core import blenderapi
from morse.core.actuator import Actuator
from morse.helpers.components import add_data, add_property

class Keyboard(Actuator):
    """
    This actuator does not require a connection with external data. It
    simply responds to the keyboard arrows to generate movement
    instructions for the robot attached.

    When parented to a robot, the user can press the arrow keys to modify the
    linear and angular velocities (V, W) of the robot.

    :kbd:`Up` forward
    :kbd:`Down` backwards
    :kbd:`Left` turn left
    :kbd:`Right` turn right
    """

    _name = "Keyboard Actuator"
    _short_desc="A 'fake' actuator that allows to move a robot from the keyboard."

    add_property('_type', 'Position', 'Type', 'string',
                 "Type of function to move the parent robot, in ['Position', i"
                 "'Velocity', 'Differential']")
    add_property('_speed', 1.0, 'Speed', 'float',
                 "Movement speed of the parent robot, in m/s")

    def __init__(self, obj, parent=None):
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        Actuator.__init__(self, obj, parent)

        # Correct the speed considering the Blender clock
        if self._type == 'Position':
            self._speed = self._speed / self.frequency
        elif self._type == 'Differential':
            self._stopped = True
            # get track width for calculating wheel speeds from yaw rate
            parent = self.robot_parent
            self._trackWidth = parent._trackWidth
            self._radius = parent._wheelRadius

        logger.info('Component initialized')


    def default_action(self):
        """ Interpret keyboard presses and assign them to movement
            for the robot."""
        keys_sensor = blenderapi.controller().sensors[0]
        #pressed_keys = keys_sensor.getPressedKeys()
        pressed_keys = keys_sensor.events

        # Reset movement variables
        vx, vy, vz = 0.0, 0.0, 0.0
        rx, ry, rz = 0.0, 0.0, 0.0

        for key, status in pressed_keys:
            logger.debug("GOT: {0}, STATUS {1}".format(key, status))
            if key == blenderapi.UPARROWKEY:
                vx = self._speed

            if key == blenderapi.DOWNARROWKEY:
                vx = -self._speed

            if key == blenderapi.LEFTARROWKEY:
                rz = self._speed

            if key == blenderapi.RIGHTARROWKEY:
                rz = -self._speed

        # Get the Blender object of the parent robot
        parent = self.robot_parent.bge_object

        # Give the movement instructions directly to the parent
        # The second parameter specifies a "local" movement
        if self._type == 'Position':
            parent.applyMovement([vx, vy, vz], True)
            parent.applyRotation([rx, ry, rz / 2.0], True)
        elif self._type == 'Velocity':
            parent.setLinearVelocity([vx, vy, vz], True)
            parent.setAngularVelocity([rx, ry, rz / 2.0], True)
        elif self._type == 'Differential':
            self.apply_vw_wheels(vx, -rz)

    # from v_omega_diff_drive
    def apply_vw_wheels(self, vx, vw):
        """ Apply (v, w) to the parent robot. """

        # calculate desired wheel speeds and set them
        if abs(vx) < 0.001 and abs(vw) < 0.001:
            # stop the wheel when velocity is below a given threshold
            for index in self.robot_parent._wheels.keys():
                self.robot_parent._wheel_joints[index].setParam(9, 0, 100.0)

            self._stopped = True
        else:
            # this is need to "wake up" the physic objects if they have
            # gone to sleep apply a tiny impulse straight down on the
            # object
            if (self._stopped):
                self.robot_parent.bge_object.applyImpulse(
                   self.robot_parent.bge_object.position, (0.0, 0.1, -0.000001))

            # no longer stopped
            self._stopped = False

            # Another formula for computing left and right wheel speeds:
            # http://arri.uta.edu/acs/jmireles/Robotics/KinematicsMobileRobots.pdf
            v_ws_l = vx - (self._trackWidth / 2.0) * vw
            v_ws_r = vx + (self._trackWidth / 2.0) * vw

            # convert to angular speeds
            w_ws_l = v_ws_l / self._radius
            w_ws_r = v_ws_r / self._radius

            # set wheel speeds - front and rear wheels have the same speed
            # Left side wheels
            self.robot_parent._wheel_joints['FL'].setParam(9, w_ws_l, 100.0)
            if 'RL' in self.robot_parent._wheels:
                self.robot_parent._wheel_joints['RL'].setParam(9, w_ws_l, 100.0)
            # Right side wheels
            self.robot_parent._wheel_joints['FR'].setParam(9, w_ws_r, 100.0)
            if 'RR' in self.robot_parent._wheels:
                self.robot_parent._wheel_joints['RR'].setParam(9, w_ws_r, 100.0)

            logger.debug("New speeds set: left=%.4f, right=%.4f" %
                         (w_ws_l, w_ws_r))

