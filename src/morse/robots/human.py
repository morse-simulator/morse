import logging; logger = logging.getLogger("morse." + __name__)
from morse.core import blenderapi
from morse.robots.grasping_robot import GraspingRobot
from morse.core.services import service

class Human(GraspingRobot):
    """ Class definition for the human as a robot entity.

    Sub class of GraspingRobot.
    """

    def __init__(self, obj, parent=None):
        """ Call the constructor of the parent class """
        logger.info('%s initialization' % obj.name)
        GraspingRobot.__init__(self, obj, parent)

        # We define here the name of the human grasping hand:
        self.hand_name = 'Hand_Grab.R'

        logger.info('Component initialized')

    @service
    def move(self, speed, rotation):
        """ Move the human. """

        human = self.bge_object

        if not human['Manipulate']:
            human.applyMovement( [speed,0,0], True )
            human.applyRotation( [0,0,rotation], True )
        else:
            scene = blenderapi.scene()
            target = scene.objects['IK_Target_Empty.R']

            target.applyMovement([0.0, rotation, 0.0], True)
            target.applyMovement([0.0, 0.0, -speed], True)

    @service
    def move_head(self, pan, tilt):
        """ Move the human head. """

        human = self.bge_object
        scene = blenderapi.scene()
        target = scene.objects['Target_Empty']

        if human['Manipulate']:
            return

        target.applyMovement([0.0, pan, 0.0], True)
        target.applyMovement([0.0, 0.0, tilt], True)

    @service
    def move_hand(self, diff, tilt):
        """ Move the human hand (wheel).

        A request to use by a socket.
        Done for wiimote remote control.
        """

        human = self.bge_object
        if human['Manipulate']:
            scene = blenderapi.scene()
            target = scene.objects['IK_Target_Empty.R']
            target.applyMovement([diff, 0.0, 0.0], True)  
        
    @service
    def toggle_manipulation(self):
        """ Change from and to manipulation mode.

        A request to use by a socket.
        Done for wiimote remote control.
        """

        human = self.bge_object
        scene = blenderapi.scene()
        hand_target = scene.objects['IK_Target_Empty.R']
        head_target = scene.objects['Target_Empty']

        if human['Manipulate']:
            human['Manipulate'] = False
            # Place the hand beside the body
            hand_target.localPosition = [0.0, -0.3, 0.8]
            head_target.setParent(human)
            head_target.localPosition = [1.3, 0.0, 1.7]
        else:
            human['Manipulate'] = True
            head_target.setParent(hand_target)
            # Place the hand in a nice position
            hand_target.localPosition = [0.6, 0.0, 1.4]
            # Place the head in the same place
            head_target.localPosition = [0.0, 0.0, 0.0]

    def default_action(self):
        """ Main function of this component. """
        pass
