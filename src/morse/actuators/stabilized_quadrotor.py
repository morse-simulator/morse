import logging; logger = logging.getLogger("morse." + __name__)
from morse.core.services import service
import morse.core.actuator
import mathutils
from morse.helpers.filt2 import Filt2

class StabilizedQuadrotorActuatorClass(morse.core.actuator.MorseActuatorClass):
    """ Motion controller using linear model of a stabilized quadrotor

    This class will read Angular and height input (phi, theta, psi, h)
    and then apply the computed velocity and angular speeds to the 
    parent robot.
    """

    def __init__(self, obj, parent=None):
        logger.info('%s initialization' %obj.name)
        # Call the constructor of the parent class
        super(self.__class__, self).__init__(obj, parent)
        # Commands
        self.local_data['phi_c'] = 0.0
        self.local_data['theta_c'] = 0.0
        self.local_data['psi_c'] = 0.0
        self.local_data['h_c'] = 0.0
        self._type = 'Position'
        # Env Variables
        self.v = [0.0, 0.0, 0.0]
        self.acc = [0.0, 0.0, 0.0]
        self.f_phi = Filt2(10.0, 0.7)
        self.f_theta = Filt2(10.0, 0.7)
        self.f_psi = Filt2(10.0, 0.7)
        self.f_h = Filt2(3.0, 0.5)
        # Local Variables

        logger.info('Component initialized')

    @service
    def set_cons(self, phi, theta, psi, h):
        self.local_data['phi_c'] = phi
        self.local_data['theta_c'] = theta
        self.local_data['psi_c'] = self.local_data['psi_c']+ psi
        self.local_data['h_c'] = h

    @service
    def stop(self):
        self.v = [0.0, 0.0, 0.0]
        self.f_phi.init()
        self.f_theta.init()
        self.f_psi.init()

    def default_action(self):
        """ Apply ... to the parent robot. """
        # Ticks
        dt = 1.0 / self.frequency

        # Compute height and Euler Angles
        self.f_phi.simulate(self.local_data['phi_c'], dt)
        self.f_theta.simulate(self.local_data['theta_c'], dt)
        self.f_psi.simulate(self.f_psi.x[0]-self.local_data['psi_c'], dt)
        self.f_h.simulate(self.local_data['h_c'], dt)
        rot = mathutils.Euler([self.f_phi.x[0], self.f_theta.x[0],
                                                self.f_psi.x[0]])

        # Get the parent
        parent = self.robot_parent.blender_obj

        # Compute acceleration
        # get previous height and vert. velocity
        main_to_origin = self.robot_parent.position_3d
        main_to_origin.update(parent)
        h =  main_to_origin.z

        # assume velocity
        acc_b = mathutils.Vector([0.0, 0.0,
                                  10.0 + 9.0 * (self.local_data['h_c'] - h) -
                                  2.0 * 0.7 * 3.0 * self.v[2]])
        acc_i = mathutils.Vector( rot.to_matrix()*acc_b )
        self.v[0] = self.v[0] + dt * acc_i[0]
        self.v[1] = self.v[1] + dt * acc_i[1]
        self.v[2] = self.v[2] + dt * (acc_i[2] -10.0)

        #Change the parent position
        prev_pos = parent.localPosition
        parent.localPosition = mathutils.Vector(prev_pos +
                dt * mathutils.Vector([self.v[0],self.v[1],self.v[2]]))
        #Change the parent orientation
        parent.orientation = rot.to_matrix()

