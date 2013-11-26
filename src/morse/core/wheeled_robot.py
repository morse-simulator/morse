import logging; logger = logging.getLogger("morse." + __name__)
from abc import ABCMeta
from math import sqrt
import morse.core.robot
from morse.core import blenderapi
from morse.core import mathutils
from morse.helpers.components import add_property

class PhysicsWheelRobot(morse.core.robot.Robot):
    """ Abstract base class for robots with wheels that turn as
        the robot moves.
        The wheels must be children of the robot in the Blender file.
        """
    # Make this an abstract class
    __metaclass__ = ABCMeta

    add_property('_has_suspension', True, 'HasSuspension', 'bool', 
                 'Determine if the underlaying robot has suspension, \
                  i.e. wheels can move independently of the body of the \
                  robot')
    add_property('_has_steering', True, 'HasSteering', 'bool',
                 'Determine if the wheels turn independently of the body \
                  of the robot.')

    # Local dictionaries to store references to the wheels
    _wheel_index = ['FL', 'FR', 'RL', 'RR']

    def __init__(self, obj, parent):
        morse.core.robot.Robot.__init__(self, obj, parent)
        self._wheels = {}
        self._wheel_positions = {}
        self._wheel_orientations = {}
        self._wheel_joints = {}

    def action(self):
        """ Overload the 'action' method of the Robot
            This one will compute the transformations considering the different
            axis orientation used by this kind of robots """
        # Update the component's position in the world
        self.position_3d.update_Y_forward(self.bge_object)

        self.default_action()


    def get_wheels(self):
        # get pointers to and physicsIds of all objects
        # get wheel pointers - needed by wheel speed sensors and to
        # set up constraints
        # bullet vehicles always have 4 wheels
        scene = blenderapi.scene()

        self._wheel_radius = None

        caster_wheel_name = self.bge_object.get('CasterWheelName', None)

        #  inherited from the parent robot
        for index in self._wheel_index:
            name = "Wheel%sName" % index
            # Get the actual name of the object from the properties
            #  of the parent robot
            try:
                wheel = scene.objects[self.bge_object[name]]
            except:
                #import traceback
                #traceback.print_exc()
                wheel = None

            if wheel:
                self._wheels[index] = wheel
                logger.info("\tWheel %s: %s" % (index, wheel.name))
                self._wheel_positions[index] = \
                    mathutils.Vector(wheel.worldPosition)
                self._wheel_orientations[index] = \
                    mathutils.Matrix(wheel.worldOrientation)
                # Make the wheels orphans
                wheel.removeParent()
                # Keep their transformations
                #wheel.worldPosition = self._wheel_positions[index]
                #wheel.worldOrientation = self._wheel_orientations[index]

                # get wheel radius if not already computed
                if wheel.name != caster_wheel_name and not self._wheel_radius:
                    self._wheel_radius = self.get_wheel_radius(self.bge_object[name])

        logger.debug("get_wheels %s" % self._wheels)

        # Add a free rotating wheel if indicated in the robot
        if caster_wheel_name:
            wheel = scene.objects[caster_wheel_name]
            wheel_position = mathutils.Vector(wheel.worldPosition)
            self.attach_caster_wheel_to_body(wheel, self.bge_object, wheel_position)

    def get_track_width(self):
        # get lateral positions of the wheels
        pos_l = self._wheel_positions['FL']
        pos_r = self._wheel_positions['FR']

        diff_x = pos_l[0] - pos_r[0]
        diff_y = pos_l[1] - pos_r[1]
        diff_z = pos_l[2] - pos_r[2]
        return sqrt( diff_x ** 2 + diff_y ** 2 + diff_z ** 2)

    def get_wheel_radius(self, wheel_name):
        dims = blenderapi.objectdata(wheel_name).dimensions
        # average the x and y dimension to get diameter - divide by 2 for radius
        radius = (dims[0]+dims[1])/4
        return radius


class MorsePhysicsRobot(PhysicsWheelRobot):
    """ Basic Class for robots using individual physics constraints

    Inherits from the base robot class.
    """

    def __init__ (self, obj, parent=None):
        """ Constructor method. """
        # Call the constructor of the parent class
        super(MorsePhysicsRobot, self).__init__(obj, parent)

        # get wheel references and ID's
        self.get_wheels()

        # construct the vehicle
        self.build_vehicle()


    def build_vehicle(self):
        """ Apply the constraints to the vehicle parts. """

        # chassis ID - main object should be chassis model
        self._chassis_ID = self.bge_object.getPhysicsId()

        # get track width
        self._trackWidth = self.get_track_width()

        # set up wheel constraints
        # add wheels to either suspension arms or vehicle chassis
        if self._has_suspension:
            self.build_model_with_suspension()
        else:
            self.build_model_without_suspension()

#    def build_model_with_suspension(self):
#        """ Add all the constraints to attach the wheels to
#        the a-arms and then the a-arms to the body """
#        scene = blenderapi.scene()
#        # get suspension arm ID's
#        # front left A-arm
#        try:
#            if self.bge_object['ArmFLName']:
#                self._armFL=scene.objects[self.bge_object['ArmFLName']]
#        except:
#            import traceback
#            traceback.print_exc()
#
#        # front right A-arm
#        try:
#            if self.bge_object['ArmFRName']:
#                self._armFR=scene.objects[self.bge_object['ArmFRName']]
#        except:
#            import traceback
#            traceback.print_exc()
#
#        # rear left arm
#        try:
#            if self.bge_object['ArmRLName']:
#                self._armRL=self.bge_object['ArmRLName']
#        except:
#            import traceback
#            traceback.print_exc()
#
#        # rear right arm
#        try:
#            if self.bge_object['ArmRRName']:
#                self._armRR=self.bge_object['ArmRRName']
#        except:
#            import traceback
#            traceback.print_exc()
#
#        # put together front wheels and suspension
#        self._wheelFLJoint=self.AttachWheelWithSuspension(self._wheelFL,self.bge_object,self._armFL)
#        self._wheelFRJoint=self.AttachWheelWithSuspension(self._wheelFR,self.bge_object,self._armFR)
#
#        self._wheelRLJoint=self.AttachWheelWithSuspension(self._wheelRL,self.bge_object,self._armRL)
#        self._wheelRRJoint=self.AttachWheelWithSuspension(self._wheelRR,self.bge_object,self._armRR)

    def build_model_without_suspension(self):
        """ Add all the constraints to attach the wheels to the body """
        for index in self._wheels.keys():
            self._wheel_joints[index] = self.attach_wheel_to_body(
                    self._wheels[index], self.bge_object,
                    self._wheel_positions[index])

    def attach_wheel_to_body(self, wheel, parent, wheel_pos):
        """ Attaches the wheel to the given parent using a 6DOF constraint

        Set the wheel positions relative to the robot in case the
        chassis was moved by the builder script or manually in blender
        """

        result = parent.getVectTo(wheel)
        ## result is a unit vector (result[2]) and a length(result[0])
        ## multiply them together to get the complete vector
        wheel_pos = result[0] * result[2]

        logger.debug("Added wheel '%s' at ('%f','%f','%f')" %
                (wheel.name, wheel_pos[0], wheel_pos[1], wheel_pos[2]))

        # create constraint to allow wheel to spin
        # For an explanation on the parameters, see:
        # http://www.tutorialsforblender3d.com/GameModule/ClassKX_PyConstraintBinding_1f.html
        joint = blenderapi.constraints().createConstraint(
                parent.getPhysicsId(),  # get physics ID of the parent object
                wheel.getPhysicsId(),   # get physics ID of the wheel object
                12,                     # 6dof constraint
                wheel_pos[0], wheel_pos[1], wheel_pos[2],  # pivot position
                0,0,0,                  # pivot axis
                128)    # flag, 128=disable collision between wheel and parent
        # no parameters are set on x axis to allow full rotation about it
        joint.setParam(4, 0.0, 0.0) # no rotation about Y axis - min=0, max=0
        joint.setParam(5, 0.0, 0.0) # no rotation about Z axis - min=0, max=0
        return joint # return a reference to the constraint

    def attach_caster_wheel_to_body(self, wheel, parent, wheel_pos):
        """ Attaches a freely rotating wheel to the given parent
        using a 6DOF constraint. It can also rotate around the Z axis """

        result = parent.getVectTo(wheel)
        ## result is a unit vector (result[2]) and a length(result[0])
        ## multiply them together to get the complete vector
        wheel_pos = result[0] * result[2]

        logger.debug("Added caster wheel '%s' at ('%f','%f','%f')" %
                (wheel.name, wheel_pos[0], wheel_pos[1], wheel_pos[2]))

        # create constraint to allow wheel to spin
        joint = blenderapi.constraints().createConstraint(
                parent.getPhysicsId(),  # get physics ID of the parent object
                wheel.getPhysicsId(),   # get physics ID of the wheel object
                12,                     # 6dof constraint
                wheel_pos[0], wheel_pos[1], wheel_pos[2],  # pivot position
                0, 0, 0,                  # pivot axis
                128)    # flag, 128=disable collision between wheel and parent
        # no parameters are set on x and z axis to allow full rotation about it
        joint.setParam(4, 0.0, 0.0) # no rotation about Y axis - min=0, max=0
        joint.setParam(5, 0.0, 0.0) # no rotation about Z axis - min=0, max=0
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
