import logging; logger = logging.getLogger("morse." + __name__)
from morse.core import blenderapi
import morse.core.robot
from morse.core.services import service

class HumanClass(morse.core.robot.MorseRobotClass):
    """ Class definition for the human as a robot entity
        Sub class of Morse_Object. """

    def __init__(self, obj, parent=None):
        """ Constructor method.
            Receives the reference to the Blender object.
            Optionally it gets the name of the object's parent,
            but that information is not currently used for a robot. """
        # Call the constructor of the parent class
        logger.info('%s initialization' % obj.name)
        super(self.__class__,self).__init__(obj, parent)

        logger.info('Component initialized')

    @service
    def move(self, speed, rotation):
        """ Move the human.
        """
        
        human = self.blender_obj
        
        if not human['Manipulate']:
            human.applyMovement( [speed,0,0], True )
            human.applyRotation( [0,0,rotation], True )
        else :
            scene = blenderapi.scene()
            target = scene.objects['IK_Target_Empty.R']

            target.applyMovement([0.0, rotation, 0.0], True)
            target.applyMovement([0.0, 0.0, -speed], True)
        
        
    @service
    def move_head(self, pan, tilt):
        """ Move the human head.
        """
        
        human = self.blender_obj
        scene = blenderapi.scene()
        target = scene.objects['Target_Empty']
        
        if human['Manipulate']:
            return

        target.applyMovement([0.0, pan, 0.0], True)
        target.applyMovement([0.0, 0.0, tilt], True)
        
    @service
    def grasp_(self, seq):
        """ Grasp object.
        """
        human = self.blender_obj
        if human['Manipulate']:
            scene = blenderapi.scene()
            hand_empty = scene.objects['Hand_Grab.R']

            selected_object = hand_empty['Near_Object']
            if seq == "t":
                # Check that no other object is being carried
                if (human['DraggedObject'] == None or 
                human['DraggedObject'] == '') :
                    # If the object is draggable
                    if selected_object != None and selected_object != '':
                        # Clear the previously selected object, if any
                        human['DraggedObject'] = selected_object
                        # Remove Physic simulation
                        selected_object.suspendDynamics()
                        # Parent the selected object to the hand target
                        selected_object.setParent (hand_empty)
                        
        if seq == "f":

            if (human['DraggedObject'] != None and 
            human['DraggedObject'] != '') :
                previous_object = human["DraggedObject"]
                # Restore Physics simulation
                previous_object.restoreDynamics()
                previous_object.setLinearVelocity([0, 0, 0])
                previous_object.setAngularVelocity([0, 0, 0])
                # Remove the parent
                previous_object.removeParent()
                # Clear the object from dragged status
                human['DraggedObject'] = None
        
    @service
    def move_hand(self, diff, tilt):
        """ move the human hand (wheel). a request to use by a socket.
        Done for wiimote remote control.
        """
        
        human = self.blender_obj
        if human['Manipulate']:
            scene = blenderapi.scene()
            target = scene.objects['IK_Target_Empty.R']
            target.applyMovement([diff, 0.0, 0.0], True)  
        
    @service
    def toggle_manipulation(self):
        """ change from and to manipulation mode. a request to use by a socket.
        Done for wiimote remote control.
        """
        
        human = self.blender_obj
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
