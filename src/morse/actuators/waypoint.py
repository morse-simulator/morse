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
from morse.core import status, blenderapi, mathutils
import morse.core.actuator
from morse.core.services import service, async_service, interruptible
from morse.helpers.components import add_data, add_property

class WaypointActuatorClass(morse.core.actuator.MorseActuatorClass):
    """ Waypoint motion controller

    This controller will receive a destination point and
    make the robot move to that location by moving forward and turning.
    This controller is meant for land robots that can not move sideways.
    """
    _name = "Waypoint"
    add_property('_obstacle_avoidance', True, 'ObstacleAvoidance', 'boolean')
    add_property('_free_z', False, 'FreeZ', 'boolean')
    add_property('_angle_tolerance', math.radians(10), 'AngleTolerance', 'float', "Tolerance in radians regarding the final heading of the robot")
    add_property('_speed', 1.0, 'Speed', 'float')

    add_data('x', 0.0, "float", "X coordinate of the destination, in world frame")
    add_data('y', 0.0, "float", "Y coordinate of the destination, in world frame")
    add_data('z', 0.0, "float", "Z coordinate of the destination, in world frame")
    add_data('tolerance', 0.5, "float", "Tolerance, in meter, to consider the destination as reached.")
    add_data('speed', 1.0, "float", "If the property 'Speed' is set, use this speed as initial value.")
 


    def __init__(self, obj, parent=None):

        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(obj, parent)

        # Direction of the global vectors
        self.world_X_vector = mathutils.Vector([1,0,0])
        self.world_Y_vector = mathutils.Vector([0,1,0])
        self.world_Z_vector = mathutils.Vector([0,0,1])

        self._destination = self.blender_obj.position
        self._projection = self.blender_obj.position
        self._wp_object = None
        self._collisions = False
        
        # Variable to store current speed. Used for the stop/resume services
        self._previous_speed = 0

        # Choose the type of function to move the object
        #self._type = 'Velocity'
        self._type = 'Position'

        self.local_data['x'] = self._destination[0]
        self.local_data['y'] = self._destination[1]
        self.local_data['z'] = self._destination[2]
        # Waypoint tolerance (in meters)
        self.local_data['tolerance'] = 0.5
        self.local_data['speed'] = self._speed

        # Identify an object as the target of the motion
        try:
            wp_name = self.blender_obj['Target']
            if wp_name != '':
                scene = blenderapi.scene()
                self._wp_object = scene.objects[wp_name]
                logger.info("Using object '%s' to indicate motion target" % wp_name)
        except KeyError as detail:
            self._wp_object = None

        # Identify the collision detectors for the sides
        if self._obstacle_avoidance:
            for child in self.blender_obj.children:
                if "Radar.R" in child.name:
                    self._radar_r = child
                if "Radar.L" in child.name:
                    self._radar_l = child
            try:
                logger.info("Radar Right is '%s'" % self._radar_r.name)
                logger.info("Radar Left  is '%s'" % self._radar_l.name)
                self._collisions = True
            except AttributeError as detail:
                logger.warning("No radars found attached to the waypoint actuator.\n\tThere will be no obstacle avoidance")

            # Convert a string listing the properties to avoid into a list
            # Objects with any of these properties will be ignored
            #  when doing obstacle avoidance
            try:
                props = self.blender_obj['Ignore'].split(',')
                self.blender_obj['Ignore'] = props
            except KeyError as detail:
                self.blender_obj['Ignore'] = []


        logger.info('Component initialized')


    @service
    def setdest(self, x, y, z, tolerance=0.5, speed=1.0):
        """ Set a new waypoint and returns.

        The robot will try to go to this position, but the service 
        caller has no mean to know when the destination is reached
        or if it failed.

        In most cases, the asynchronous service 'goto' should be 
        prefered.

        :return: Returns always True (if the robot is already moving, the
        previous target is replaced by the new one) except if
        the destination is already reached. In this case, returns
        False.
        """

        distance, gv, lv = self.blender_obj.getVectTo([x,y,z])
        if distance - tolerance <= 0:
            logger.info("Robot already at destination (distance = {})."
                    " I do not set a new destination.".format(distance))
            return False

        self.local_data['x'] = x
        self.local_data['y'] = y
        self.local_data['z'] = z
        self.local_data['tolerance'] = tolerance
        self.local_data['speed'] = speed

        return True


    @interruptible
    @async_service
    def goto(self, x, y, z, tolerance=0.5, speed=1.0):
        """ Go to a new destination.

        The service returns when the destination is reached within
        the provided tolerance bounds.
        """
        self.local_data['x'] = x
        self.local_data['y'] = y
        self.local_data['z'] = z
        self.local_data['tolerance'] = tolerance
        self.local_data['speed'] = speed


    @service
    #@async_service
    def stop(self):
        """ Interrup the movement of the robot """
        #self.local_data['x'] = self.blender_obj.worldPosition[0]
        #self.local_data['y'] = self.blender_obj.worldPosition[1]
        #self.local_data['z'] = self.blender_obj.worldPosition[2]
        #self.local_data['tolerance'] = 0.5
        self._previous_speed = self.local_data['speed']
        self.local_data['speed'] = 0

        # Set the status of the robot
        self.robot_parent.move_status = "Stop"

        return self.robot_parent.move_status

    @service
    def resume(self):
        """ Restore the previous speed and keep going towards the waypoint """
        self.local_data['speed'] = self._previous_speed
        self._previous_speed = 0

        # Set the status of the robot
        self.robot_parent.move_status = "Transit"

        return self.robot_parent.move_status


    @service
    def get_status(self):
        """ Return the current status (Transit, Arrived or Stop) """
        return self.robot_parent.move_status


    def default_action(self):
        """ Move the object towards the destination. """
        parent = self.robot_parent
        speed = self.local_data['speed']
        v = 0
        rz = 0

        self._destination = [ self.local_data['x'], self.local_data['y'], self.local_data['z'] ]
        self._projection = [ self.local_data['x'], self.local_data['y'], self.blender_obj.worldPosition[2] ]
        
        logger.debug("Robot {0} move status: '{1}'".format(parent.blender_obj.name, parent.move_status))
        # Place the target marker where the robot should go
        if self._wp_object:
            self._wp_object.position = self._destination

        # Vectors returned are already normalized
        projection_distance, projection_vector, local_vector = self.blender_obj.getVectTo(self._projection)
        true_distance, global_vector, local_vector = self.blender_obj.getVectTo(self._destination)
        # Convert to the Blender Vector object
        global_vector = mathutils.Vector(global_vector)
        projection_vector = mathutils.Vector(projection_vector)
        # if Z is not free, distance is the projection distance
        if self._free_z:
            distance = true_distance
        else:
            distance = projection_distance
        
        logger.debug("GOT DISTANCE: xy: %.4f ; xyz: %.4f" % (projection_distance, true_distance))
        logger.debug("Global vector: %.4f, %.4f, %.4f" % (global_vector[0], global_vector[1], global_vector[2]))
        logger.debug("Local vector: %.4f, %.4f, %.4f" % (local_vector[0], local_vector[1], local_vector[2]))
        logger.debug("Projection vector: %.4f, %.4f, %.4f" % (projection_vector[0], projection_vector[1], projection_vector[2]))

        # If the target has been reached, change the status
        if distance - self.local_data['tolerance'] <= 0:
            parent.move_status = "Arrived"

            #Do we have a running request? if yes, notify the completion
            self.completed(status.SUCCESS, parent.move_status)

            logger.debug("TARGET REACHED")
            logger.debug("Robot {0} move status: '{1}'".format(parent.blender_obj.name, parent.move_status))

        else:
            # Do nothing if the speed is zero
            if speed == 0:
                return

            parent.move_status = "Transit"

            angle_diff = 0
            rotation_direction = 0

            # If the projected distance is not null: else computing the target angle is not possible!
            if projection_distance - self.local_data['tolerance'] / 2 > 0:
                ### Get the angle of the robot ###
                robot_angle = parent.position_3d.yaw

                ### Get the angle to the target ###
                target_angle = projection_vector.angle(self.world_X_vector)

                # Correct the direction of the turn according to the angles
                dot = projection_vector.dot(self.world_Y_vector)
                logger.debug("Vector dot product = %.2f" % dot)
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

            try:
                # Compute the speeds
                if self._type == 'Position':
                    v = speed / self.frequency
                    rotation_speed = (speed / self.frequency) / 2.0
                elif self._type == 'Velocity':
                    v = speed
                    rotation_speed = 1.0 #speed / 2.0
            # For the moment ignoring the division by zero
            # It happens apparently when the simulation starts
            except ZeroDivisionError:
                pass

            # Allow the robot to rotate in place if the waypoing is
            #  to the side or behind the robot
            if angle_diff >= math.pi/3.0:
                logger.debug("Turning on the spot!!!")
                v = 0

            # Collision avoidance using the Blender radar sensor
            if self._collisions and v != 0 and self._radar_r['Rcollision']:
                # No obstacle avoidance when the waypoint is near
                if distance+self.local_data['tolerance'] > self._radar_r.sensors["Radar"].distance:
                    # Ignore obstacles with the properties specified
                    ignore = False
                    for prop in self.blender_obj['Ignore']:
                        if prop in self._radar_r.sensors["Radar"].hitObject:
                            ignore = True
                            logger.debug("Ignoring object '%s' with property '%s'" % (self._radar_r.sensors["Radar"].hitObject, prop))
                            break
                    if not ignore:
                        rz = rotation_speed
                        logger.debug("Obstacle detected to the RIGHT, turning LEFT")
            elif self._collisions and v != 0 and self._radar_l['Lcollision']:
                # No obstacle avoidance when the waypoint is near
                if distance+self.local_data['tolerance'] > self._radar_l.sensors["Radar"].distance:
                    # Ignore obstacles with the properties specified
                    ignore = False
                    for prop in self.blender_obj['Ignore']:
                        if prop in self._radar_l.sensors["Radar"].hitObject:
                            ignore = True
                            logger.debug("Ignoring object '%s' with property '%s'" % (self._radar_l.sensors["Radar"].hitObject, prop))
                            break
                    if not ignore:
                        rz = - rotation_speed
                        logger.debug("Obstacle detected to the LEFT, turning RIGHT")
            # Test if the orientation of the robot is within tolerance
            elif -self._angle_tolerance < angle_diff < self._angle_tolerance:
                rz = 0
            # If not, rotate the robot in the corresponding direction
            else:
                rz = rotation_speed * rotation_direction

            if self._free_z:
                vx = math.fabs(v * local_vector.dot(self.world_X_vector))
                vz = v * local_vector.dot(self.world_Z_vector)
            else:
                vx = v
                vz = 0
            logger.debug("Applying [[vx = %.4f, vz = %.4f, rz = %.4f]] (v = %.4f)\n" % (vx, vz, rz, v))

            # Give the movement instructions directly to the parent
            # The second parameter specifies a "local" movement
            if self._type == 'Position':
                parent.blender_obj.applyMovement([vx, 0, vz], True)
                parent.blender_obj.applyRotation([0, 0, rz], True)
            elif self._type == 'Velocity':
                parent.blender_obj.setLinearVelocity([vx, 0, vz], True)
                parent.blender_obj.setAngularVelocity([0, 0, rz], True)
