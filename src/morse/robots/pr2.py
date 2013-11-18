import logging; logger = logging.getLogger("morse." + __name__)
from morse.robots.grasping_robot import GraspingRobot
from morse.core.services import service
from morse.core import blenderapi

class PR2(GraspingRobot):
    """ 
    The MORSE model of the Willow Garage's PR2 robot.

    The PR2 uses the :doc:`armature_actuator <../actuators/armature>`
    for control of the armatures.

    Model Info
    ----------

    The model is imported from a Collada file that is generated from the
    `PR2 URDF file  <http://www.ros.org/wiki/pr2_description>`_.
    The .dae file can be found at:
    ``$MORSE_ROOT/data/robots/pr2/pr2.dae``
    The imported .blend file can be found at:
    ``$MORSE_ROOT/data/robots/pr2/pr2_25_original.blend``

    The URDF to Collada converter changed all the object names, so these
    were remapped to the original URDF names. A renamed version of the
    PR2 model can be found at:
    ``$MORSE_ROOT/data/robots/pr2/pr2_25_rename.blend`` , this file
    includes the script that is used to rename all the objects.

    A model with MORSE integration for the armature can be found at
    (**This is the model that you probably want to use in MORSE**):
    ``$MORSE_ROOT/data/robots/pr2/pr2_25_morse.blend``.

    TODO
    ----

    - Create sensors and actuators to control the PR2 armature. `A
      SensorActuator class would be handy for this
      <https://sympa.laas.fr/sympa/arc/morse-users/2011-07/msg00099.html>`_.
    - Expand the armature to include the hands.
    - Add an actuator to control the movement of the PR2 base.
    - ROS integration.
    - ...

    """

    _name = 'PR2 robot'

    def __init__(self, obj, parent=None):
        """ 
        Constructor method.
        Receives the reference to the Blender object.
        """
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        GraspingRobot.__init__(self, obj, parent)

        """
        We define here the name of the pr2 grasping hand:
        """
        self.hand_name = 'Hand.Grasp.PR2'

        self.armatures = []
        # Search armatures and torso in all objects parented to the pr2 empty
        for obj in self.bge_object.childrenRecursive:
            # Check if obj is an armature
            if type(obj).__name__ == 'BL_ArmatureObject':
                self.armatures.append(obj.name)
            if obj.name == 'torso_lift_joint':
                self.torso = obj

        # constant that holds the original height of the torso from the ground
        # These values come from the pr2 urdf file
        self.TORSO_BASE_HEIGHT = (0.739675 + 0.051)
        self.TORSO_LOWER = 0.0  # lower limit on the torso z-translantion
        self.TORSO_UPPER = 0.31  # upper limit on the torso z-translation
        
        logger.info('Component initialized')

    
    @service
    def get_armatures(self):
        """
        Returns a list of all the armatures on the PR2 robot.
        """
        return self.armatures

    @service
    def set_torso(self, height):
        """
        MORSE Service that sets the z-translation of the torso to original_z + height.
        """
        if self.TORSO_LOWER < height < self.TORSO_UPPER:
            self.torso.localPosition = [-0.05, 0, self.TORSO_BASE_HEIGHT + height]
            return "New torso z position: " + str(self.torso.localPosition[2])
        else:
            return "Not a valid height, value has to be between 0.0 and 0.31!"
            
    @service
    def get_torso(self):
        """
        Returns the z-translation of the torso.
        """
        return self.torso.localPosition[2] - self.TORSO_BASE_HEIGHT

    @service
    def get_torso_minmax(self):
        """
        Returns the minimum an maximum z-translation that the torso can
        make from the base.  Returns a list [min,max]
        """
        return [self.TORSO_LOWER, self.TORSO_UPPER]

    def default_action(self):
        """
        Main function of this component.
        """
        pass
