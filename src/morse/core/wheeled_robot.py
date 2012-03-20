import logging; logger = logging.getLogger("morse." + __name__)
from abc import ABCMeta
import morse.core.robot
import bge
import bpy
import mathutils

class FourWheelRobotClass(morse.core.robot.MorseRobotClass):
    """ Abstract base class for robots with wheels that turn as
        the robot moves.
        The wheels must be children of the robot in the Blender file.
        """
    # Make this an abstract class
    __metaclass__ = ABCMeta

    # Local dictionaries to store references to the wheels
    _wheel_index = ['FL', 'FR', 'RL', 'RR']
    _wheels = {}
    _wheel_positions = {}
    _wheel_orientations = {}
    _wheel_joints = {}

    def action(self):
        """ Overload the 'action' method of the MorseRobotClass
            This one will compute the transformations considering the different
            axis orientation used by this kind of robots """
        # Update the component's position in the world
        self.position_3d.update_Y_forward(self.blender_obj)

        self.default_action()


    def GetWheels(self):
        # get pointers to and physicsIds of all objects
        # get wheel pointers - needed by wheel speed sensors and to
        # set up constraints
        # bullet vehicles always have 4 wheels
        scene=bge.logic.getCurrentScene()

        #  inherited from the parent robot
        for index in self._wheel_index:
            name = "Wheel%sName" % index
            # Get the actual name of the object from the properties
            #  of the parent robot
            try:
                wheel = scene.objects[self.blender_obj[name]]
            except:
                import traceback
                traceback.print_exc()
            self._wheels[index] = wheel
            self._wheel_positions[index] = mathutils.Vector(wheel.worldPosition)
            self._wheel_orientations[index] = mathutils.Matrix(wheel.worldOrientation)
            # Make the wheels orphans
            wheel.removeParent()
            # Keep their transformations
            #wheel.worldPosition = self._wheel_positions[index]
            #wheel.worldOrientation = self._wheel_orientations[index]

        # get wheel radius
        self._wheelRadius=self.GetWheelRadius(self.blender_obj['WheelFLName'])


    def ReadGenericParameters(self):
        # get needed parameters from the blender object
        # determines if vehicle has suspension or just wheels
        try:
            self._HasSuspension=self.blender_obj['HasSuspension']
        except KeyError as e:
            self._HasSuspension=True
            logger.info('HasSuspension property not present and defaulted to True')
        except:
            import traceback
            traceback.print_exc()

        # determines if vehicle has steerable front wheels or not
        try:
            self._HasSteering=self.blender_obj['HasSteering']
        except KeyError as e:
            self._HasSteering=True
            logger.info('HasSteering property not present and defaulted to True')
        except:
            import traceback
            traceback.print_exc()

    def GetTrackWidth(self):
        # get lateral positions of the wheels
        posL = self._wheel_positions['FL']
        posR = self._wheel_positions['FR']
        # subtract y coordinates of wheels to get width
        return posL[1]-posR[1]

    def GetWheelRadius(self, wheelName):
        dims=bpy.data.objects[wheelName].dimensions
        # average the x and y dimension to get diameter - divide by 2 for radius
        return (dims[0]+dims[1])/4


class MorsePhysicsRobotClass(FourWheelRobotClass):
    """ Basic Class for robots using individual physics constraints

    Inherits from the base robot class.
    """

    def __init__ (self, obj, parent=None):
        """ Constructor method. """
        # Call the constructor of the parent class
        super(MorsePhysicsRobotClass, self).__init__(obj, parent)

        # get wheel references and ID's
        self.GetWheels()

        # construct the vehicle
        self.build_vehicle()


    def build_vehicle(self):
        """ Apply the constraints to the vehicle parts. """

        # get a link to the blender scene to look for wheel and suspension objectsscene = GameLogic.getCurrentScene()
        # get needed parameters from the blender object
        self.ReadGenericParameters()

        # chassis ID - main object should be chassis model
        self._chassis_ID = self.blender_obj.getPhysicsId()

        # get track width
        self._trackWidth=self.GetTrackWidth();

        # set up wheel constraints
        # add wheels to either suspension arms or vehicle chassis
        if (self._HasSuspension):
            self.BuildModelWithSuspension()
        else:
            self.BuildModelWithoutSuspension()

#    def BuildModelWithSuspension(self):
#        """ Add all the constraints to attach the wheels to
#        the a-arms and then the a-arms to the body """
#        scene = GameLogic.getCurrentScene()
#        # get suspension arm ID's
#        # front left A-arm
#        try:
#            if self.blender_obj['ArmFLName']:
#                self._armFL=scene.objects[self.blender_obj['ArmFLName']]
#        except:
#            import traceback
#            traceback.print_exc()
#
#        # front right A-arm
#        try:
#            if self.blender_obj['ArmFRName']:
#                self._armFR=scene.objects[self.blender_obj['ArmFRName']]
#        except:
#            import traceback
#            traceback.print_exc()
#
#        # rear left arm
#        try:
#            if self.blender_obj['ArmRLName']:
#                self._armRL=self.blender_obj['ArmRLName']
#        except:
#            import traceback
#            traceback.print_exc()
#
#        # rear right arm
#        try:
#            if self.blender_obj['ArmRRName']:
#                self._armRR=self.blender_obj['ArmRRName']
#        except:
#            import traceback
#            traceback.print_exc()
#
#        # put together front wheels and suspension
#        self._wheelFLJoint=self.AttachWheelWithSuspension(self._wheelFL,self.blender_obj,self._armFL)
#        self._wheelFRJoint=self.AttachWheelWithSuspension(self._wheelFR,self.blender_obj,self._armFR)
#
#        self._wheelRLJoint=self.AttachWheelWithSuspension(self._wheelRL,self.blender_obj,self._armRL)
#        self._wheelRRJoint=self.AttachWheelWithSuspension(self._wheelRR,self.blender_obj,self._armRR)

    def BuildModelWithoutSuspension(self):
        """ Add all the constraints to attach the wheels to the body """
        for index in self._wheel_index:
            self._wheel_joints[index] = self.AttachWheelToBody(self._wheels[index], self.blender_obj, self._wheel_positions[index])

    def AttachWheelToBody(self, wheel, parent, wheelPos):
        """ Attaches the wheel to the given parent using a 6DOF constraint """
        # set the wheel positions relative to the robot in case the
        # chassis was moved by the builder script or manually in blender
        """
        globalWheelPos=wheelPos+parent.worldPosition
        wheel.worldPosition=globalWheelPos
        """

        result = parent.getVectTo(wheel);
        ## result is a unit vector (result[2]) and a length(result[0])
        ## multiply them together to get the complete vector
        wheelPos=result[0]*result[2]

        logger.debug("Added wheel '%s' at ('%f','%f','%f')" %(wheel.name, wheelPos[0], wheelPos[1], wheelPos[2]))

        # create constraint to allow wheel to spin
        joint = bge.constraints.createConstraint(
                parent.getPhysicsId(),  # get physics ID of the parent object
                wheel.getPhysicsId(),  # get physics ID of the wheel object
                12,    # 6dof constraint
                wheelPos[0], wheelPos[1], wheelPos[2],  # pivot position
                0,0,0,     # pivot axis
                128) # flag, 128=disable collision between wheel and parent
        # no parameters are set on x axis to allow full rotation about it
        joint.setParam(4,0.0,0.0) # no rotation about Y axis - min=0, max=0
        joint.setParam(5,0.0,0.0) # no rotation about Z axis - min=0, max=0
        return joint # return a reference to the constraint

    def AttachWheelWithSuspension(self, wheel, parent, suspensionArm):
        """ Attaches the wheel to the a-arm and then the a-arm to the body """
        # TODO: fill this in later - model after Bueraki code
        pass

#    def getWheelSpeeds(self):
#        """ Returns the angular wheel velocity in rad/sec"""
#        # true parameters tell it velocities are local
#        # wheels should be rotating about local Z axis
#        wsFL=self._wheelFL.getAngularVelocity(True)
#        wsFR=self._wheelFR.getAngularVelocity(True)
#        wsRL=self._wheelRL.getAngularVelocity(True)
#        wsRR=self._wheelRR.getAngularVelocity(True)
#        return [wsFL[2], wsFR[2], wsRL[2], wsRR[2]]
#
#    def getWheelAngle(self):
#        """ Returns the accumulated wheel angle in radians"""
#        # true parameters tell it velocities are local
#        # wheels should be rotating about local Z axis
#        wcFL=self._wheelFL.localOrientation.to_euler()
#        wcFR=self._wheelFR.localOrientation.to_euler()
#        wcRL=self._wheelRL.localOrientation.to_euler()
#        wcRR=self._wheelRR.localOrientation.to_euler()
#        return [wcFL[1], wcFR[1], wcRL[1], wcRR[1]]
#
#    def AttachWheelToWheel(self,wheel1,wheel2):
#        # add both wheels on each side to each other but with no
#        # constraints on their motion so that no collision can be set
#        # between them
#        pass
