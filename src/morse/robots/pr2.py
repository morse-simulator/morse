import logging; logger = logging.getLogger("morse." + __name__)
import morse.core.robot
from morse.core.services import service
from morse.core import blenderapi

logger.setLevel(logging.DEBUG)

class PR2Class(morse.core.robot.Robot):
    """ 
    Class definition for the PR2.
    Sub class of Morse_Object.

    This class has many MORSE Services that you can access via sockets/telnet.
    """

    def __init__(self, obj, parent=None):
        """ 
        Constructor method.
        Receives the reference to the Blender object.
        """
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(obj, parent)

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
        MORSE Service that returns a list of all the armatures on the PR2 robot.
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
        MORSE Service that returns the z-translation of the torso.
        """
        return self.torso.localPosition[2] - self.TORSO_BASE_HEIGHT

    @service
    def get_torso_minmax(self):
        """
        MORSE Service that returns the minimum an maximum z-translation that the torso can make from the base.
        Returns a list [min,max]
        """
        return [self.TORSO_LOWER, self.TORSO_UPPER]


    @service
    def grasp_(self, seq):
        """ Grasp object.
        """
        logger.debug("morse grasp request received")
        pr2 = self.bge_object
        scene = blenderapi.scene()
        hand_empty = scene.objects['Hand.Grasp.PR2']

        near_sensor = hand_empty.sensors['Near']
        near_object = near_sensor.hitObject
        hand_empty['Near_Object'] = near_object

        selected_object = hand_empty['Near_Object']
        if seq == "t":
            logger.debug("seq t")
            # Check that no other object is being carried
            if (pr2['DraggedObject'] == None or
            pr2['DraggedObject'] == '') :
                logger.debug("Hand is free, I can grab")
                # If the object is draggable
                if selected_object != None and selected_object != '':
                    # Clear the previously selected object, if any
                    logger.debug("Object to grab is %s" % selected_object.name)
                    pr2['DraggedObject'] = selected_object
                    # Remove Physic simulation
                    selected_object.suspendDynamics()
                    # Parent the selected object to the hand target
                    selected_object.setParent (hand_empty)
                    logger.debug ("OBJECT %s PARENTED TO %s" % (selected_object.name, hand_empty.name))

        if seq == "f":
            if (pr2['DraggedObject'] != None and
            pr2['DraggedObject'] != '') :
                previous_object = pr2["DraggedObject"]
                # Restore Physics simulation
                previous_object.restoreDynamics()
                previous_object.setLinearVelocity([0, 0, 0])
                previous_object.setAngularVelocity([0, 0, 0])
                # Remove the parent
                previous_object.removeParent()
                # Clear the object from dragged status
                pr2['DraggedObject'] = None
                logger.debug ("JUST DROPPED OBJECT %s" % (previous_object.name))

    def default_action(self):
        """
        Main function of this component.
        """
        pass
