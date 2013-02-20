import logging; logger = logging.getLogger("morse." + __name__)
from morse.core.services import service
import morse.core.actuator
from morse.helpers.components import add_data

class MotionVWDiff(morse.core.actuator.Actuator):
    """
    This actuator reads the values of linear and angular speed and
    applies them to the robot as speeds for the wheels. It only works
    with robots of the type ``WheeledRobot``, such as the :doc:`Segway
    RMP 400 <../robots/segwayrmp400>` and the :doc:`Pioneer 3-DX
    <../robots/pioneer3dx>`.  The movement of the robot is more
    realistic, but also depends on more factors, such as the friction
    between the wheels and the surface.

    The speeds for the left and right wheels are calculated as:

            left_speed = (v - e w) / R
            right_speed = (v + e w) / R

    where:

        - **v** is the linear velocity given as parameter
        - **w** is the angular velocity given as parameter
        - **e** is half of the distance between the left and
          right wheels
        - **R** is the radius of the wheels
    """

    _name = 'Differential Driver Actuator: \
             Linear and angular speed (V, W) actuator'

    add_data('v', 0.0, 'float',
             'linear velocity in x direction (forward movement) (m/s)')
    add_data('w', 0.0, 'float', 'angular velocity (rad/s)')

    def __init__(self, obj, parent=None):
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__, self).__init__(obj, parent)

        self.local_data['v'] = 0.0
        self.local_data['w'] = 0.0

        self._stopped = True

        # get track width for calculating wheel speeds from yaw rate
        parent = self.robot_parent
        self._trackWidth = parent._trackWidth
        self._radius = parent._wheelRadius

        logger.info('Component initialized')

    @service
    def set_speed(self, v, w):
        """
        Modifies v and w according to the parameters

        :param v: desired linear velocity (meter by second)
        :param w: desired angular velocity (radian by second)
        """
        self.local_data['v'] = v
        self.local_data['w'] = w

    @service
    def stop(self):
        """
        Stop the robot

        Internally, it sets (v, w) to (0.0, 0.0)
        """
        self.local_data['v'] = 0.0
        self.local_data['w'] = 0.0

    def default_action(self):
        """ Apply (v, w) to the parent robot. """

        # calculate desired wheel speeds and set them
        if (abs(self.local_data['v']) < 0.001) and \
           (abs(self.local_data['w']) < 0.001):
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
            v_ws_l = self.local_data['v'] - \
                     (self._trackWidth / 2.0) * self.local_data['w']
            v_ws_r = self.local_data['v'] + \
                     (self._trackWidth / 2.0) * self.local_data['w']


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
