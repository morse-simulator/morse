######################################################
#
#    waypoint.py        Blender 2.5x
#
#    A script to control the movement of a robot given
#    a destination point in space.
#    The movement includes a basic obstacle avoidance
#    system, based on the demo by Sebastian Korczak
#               admin@myinventions.pl
#
#
#    Gilberto Echeverria
#    27 / 01 / 2011
#
######################################################


import logging; logger = logging.getLogger("morse." + __name__)
import math
import GameLogic
import mathutils
import morse.core.actuator
from morse.core.services import service
from morse.core.services import async_service
from morse.core import status

class WaypointActuatorClass(morse.core.actuator.MorseActuatorClass):
    """ Waypoint motion controller

    This controller will receive a destination point and
    make the robot move to that location by moving forward and turning.
    This controller is meant for land robots that can not move sideways.
    """

    def __init__(self, obj, parent=None):

        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(obj, parent)

        # Direction of the global vectors
        self.world_X_vector = mathutils.Vector([1,0,0])
        self.world_Y_vector = mathutils.Vector([0,1,0])

        self._destination = self.blender_obj.position
        self._wp_object = None
        self._collisions = False

        # Convert the direction tolerance to radians
        self._angle_tolerance = math.radians(10)

        # Choose the type of function to move the object
        #self._type = 'Velocity'
        self._type = 'Position'

        self.local_data['x'] = self._destination[0]
        self.local_data['y'] = self._destination[1]
        self.local_data['z'] = self._destination[2]
        # Waypoint tolerance (in meters)
        self.local_data['tolerance'] = 0.5
        # Read the speed from the Blender object properties
        try:
            self.local_data['speed'] = self.blender_obj['Speed']
            logger.info("Using specified speed of %d" % self.local_data['speed'])
        # Otherwise use a default value
        except KeyError as detail:
            self.local_data['speed'] = 1.0
            logger.info("Using default speed of %d" % self.local_data['speed'])

        # Identify an object as the target of the motion
        try:
            wp_name = self.blender_obj['Target']
            if wp_name != '':
                scene = GameLogic.getCurrentScene()
                self._wp_object = scene.objects[wp_name]
                logger.info("Using object '%s' to indicate motion target" % wp_name)
        except KeyError as detail:
            self._wp_object = None

        # Identify the collision detectors for the sides
        for child in self.blender_obj.children:
            if "Radar.R" in child.name:
                self._radar_r = child
            if "Radar.L" in child.name:
                self._radar_l = child

        try:
            logger.info("Radar R is %s", self._radar_r)
            logger.info("Radar L is %s", self._radar_l)
            self._collisions = True
        except AttributeError as detail:
            logger.warning("No radars found attached to the waypoint actuator.\n\tThere will be no obstacle avoidance")

        logger.info('Component initialized')



    @async_service
    def goto(self, x, y, z, tolerance=0.5, speed=1.0):
        """ Provide new coordinates for the waypoint destination """
        #self._set_service_callback()
        self.local_data['x'] = x
        self.local_data['y'] = y
        self.local_data['z'] = z
        self.local_data['tolerance'] = tolerance
        self.local_data['speed'] = speed


    @service
    #@async_service
    def stop(self):
        """ Interrup the movement of the robot """
        #self._set_service_callback()
        self.local_data['x'] = self.blender_obj.worldPosition[0]
        self.local_data['y'] = self.blender_obj.worldPosition[1]
        self.local_data['z'] = self.blender_obj.worldPosition[2]
        self.local_data['tolerance'] = 0.5

        return self.robot_parent.move_status


    @service
    def get_status(self):
        """ Return the current status (Transit or Stop) """
        return self.robot_parent.move_status


    def default_action(self):
        """ Move the object towards the destination. """
        parent = self.robot_parent
        speed = self.local_data['speed']
        vx = 0
        rz = 0

        self._destination = [ self.local_data['x'], self.local_data['y'], self.local_data['z'] ]

        logger.debug("Robot {0} move status: '{1}'".format(parent.blender_obj.name, parent.move_status))
        # Place the target marker where the robot should go
        if self._wp_object:
            self._wp_object.position = self._destination

        # Set the z coordiante of the destination equal to that of the robot
        #  to avoid problems with the terrain.
        self._destination[2] = self.blender_obj.worldPosition[2]

        # Vectors returned are already normalised
        distance, global_vector, local_vector = self.blender_obj.getVectTo(self._destination)
        # Convert to the Blender Vector object
        global_vector = mathutils.Vector(global_vector)

        logger.debug("GOT DISTANCE: %.4f" % (distance))
        logger.debug("Global vector: %.4f, %.4f, %.4f" % (global_vector[0], global_vector[1], global_vector[2]))
        logger.debug("Local  vector: %.4f, %.4f, %.4f" % (global_vector[0], global_vector[1], global_vector[2]))

        # If the target has been reached, change the status
        if distance-self.local_data['tolerance'] <= 0:

            parent.move_status = "Stop"

            #Do we have a runing request? if yes, notify the completion
            self.completed(status.SUCCESS, parent.move_status)

            logger.debug("TARGET REACHED")
            #logger.debug("Robot {0} move status: '{1}'".format(parent, robot_state_dict['moveStatus']))

        else:
            parent.move_status = "Transit"

            ### Get the angle of the robot ###
            robot_angle = parent.position_3d.yaw

            ### Get the angle to the target ###
            target_angle = global_vector.angle(self.world_X_vector)

            # Correct the direction of the turn according to the angles
            dot = global_vector.dot(self.world_Y_vector)
            logger.debug("DOT = {0}".format(dot))
            if dot < 0:
                target_angle = target_angle * -1

            ### Get the angle that the robot must turn ###
            if target_angle < robot_angle:
                angle_diff = robot_angle - target_angle
                rotation_direction = -1
            else:
                angle_diff = target_angle - robot_angle
                rotation_direction = 1

            # Make a correction when the angles change signs
            if angle_diff > math.pi:
                angle_diff = (2 * math.pi) - angle_diff
                rotation_direction = rotation_direction * -1

            logger.debug("Angles: R=%.4f, T=%.4f  Diff=%.4f  Direction = %d" % (robot_angle, target_angle, angle_diff, rotation_direction))

            # Tick rate is the real measure of time in Blender.
            # By default it is set to 60, regardles of the FPS
            # If logic tick rate is 60, then: 1 second = 60 ticks
            ticks = GameLogic.getLogicTicRate()
            try:
                # Compute the speeds
                if self._type == 'Position':
                    vx = speed / ticks
                    rotation_speed = (speed / ticks) / 2.0
                elif self._type == 'Velocity':
                    vx = speed
                    rotation_speed = 1.0 #speed / 2.0
            # For the moment ignoring the division by zero
            # It happens apparently when the simulation starts
            except ZeroDivisionError:
                pass

            # Collision avoidance using the Blender radar sensor
            if self._collisions and self._radar_r['Rcollision']:
                rz = rotation_speed
            elif self._collisions and self._radar_l['Lcollision']:
                rz = - rotation_speed
            # Test if the orientation of the robot is within tolerance
            elif -self._angle_tolerance < angle_diff < self._angle_tolerance:
                rz = 0
            # If not, rotate the robot in the corresponding direction
            else:
                rz = rotation_speed * rotation_direction

        # Give the movement instructions directly to the parent
        # The second parameter specifies a "local" movement
        if self._type == 'Position':
            parent.blender_obj.applyMovement([vx, 0, 0], True)
            parent.blender_obj.applyRotation([0, 0, rz], True)
        elif self._type == 'Velocity':
            parent.blender_obj.setLinearVelocity([vx, 0, 0], True)
            parent.blender_obj.setAngularVelocity([0, 0, rz], True)
