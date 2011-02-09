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


import math
import GameLogic
import mathutils
import morse.helpers.actuator

class WaypointActuatorClass(morse.helpers.actuator.MorseActuatorClass):
    """ Waypoint motion controller

    This controller will receive a destination point and
    make the robot move to that location by moving forward and turning.
    This controller is meant for land robots that can not move sideways.
    """

    def __init__(self, obj, parent=None):

        print ('######## CONTROL INITIALIZATION ########')
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(obj, parent)

        # Waypoint tolerance (in meters)
        self.tolerance = 0.5
        # Convert the direction tolerance to radians
        self.angle_tolerance = math.radians(10)

        self.destination = self.blender_obj.position
        self.in_motion = False

        # Direction of the global vectors
        self.world_X_vector = mathutils.Vector([1,0,0])
        self.world_Y_vector = mathutils.Vector([0,1,0])

        self.local_data['x'] = self.destination[0]
        self.local_data['y'] = self.destination[1]
        self.local_data['z'] = self.destination[2]
        self.local_data['speed'] = 1.0
        self._wp_object = None

        # Identify an object as the target of the motion
        try:
            wp_name = self.blender_obj['Target']
            if wp_name != '':
                scene = GameLogic.getCurrentScene()
                self._wp_object = scene.objects[wp_name]
                print ("Using object '%s' to indicate motion target" % wp_name)
        except KeyError as detail:
            self._wp_object = None

        # Identify the collision detectors for the sides
        for child in self.blender_obj.children:
            if "Radar.R" in child.name:
                self._radar_r = child
                print ("Radar R is ", self._radar_r)
            if "Radar.L" in child.name:
                self._radar_l = child
                print ("Radar L is ", self._radar_l)

        print ('######## CONTROL INITIALIZED ########')


    def default_action(self):
        """ Move the object towards the destination. """
        parent = self.robot_parent
        speed = self.local_data['speed']
        vx = 0
        rz = 0

        self.destination = [ self.local_data['x'], self.local_data['y'], self.local_data['z'] ]

        #print ("Robot {0} move status: '{1}'".format(parent.blender_obj.name, parent.move_status))
        # Place the target marker where the robot should go
        if self._wp_object:
            self._wp_object.position = self.destination

        # Set the z coordiante of the destination equal to that of the robot
        #  to avoid problems with the terrain.
        self.destination[2] = self.blender_obj.worldPosition[2]

        # Vectors returned are already normalised
        distance, global_vector, local_vector = self.blender_obj.getVectTo(self.destination)
        # Convert to the Blender Vector object
        global_vector = mathutils.Vector(global_vector)

        #print ("GOT DISTANCE: %.4f" % (distance))
        #print ("Global vector: %.4f, %.4f, %.4f" % (global_vector[0], global_vector[1], global_vector[2]))
        #print ("Local  vector: %.4f, %.4f, %.4f" % (global_vector[0], global_vector[1], global_vector[2]))

        # If the target has been reached, change the status
        if distance-self.tolerance <= 0:
            parent.move_status = "Stop"
            #print ("TARGET REACHED")
            #print ("Robot {0} move status: '{1}'".format(parent, robot_state_dict['moveStatus']))

        else:
            parent.move_status = "Transit"

            ### Get the angle of the robot ###
            robot_angle = parent.position_3d.yaw

            ### Get the angle to the target ###
            target_angle = global_vector.angle(self.world_X_vector)

            # Correct the direction of the turn according to the angles
            dot = global_vector.dot(self.world_Y_vector)
            #print ("DOT = {0}".format(dot))
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

            #print ("Angles: R=%.4f, T=%.4f  Diff=%.4f  Direction = %d" % (robot_angle, target_angle, angle_diff, rotation_direction))

            # Tick rate is the real measure of time in Blender.
            # By default it is set to 60, regardles of the FPS
            # If logic tick rate is 60, then: 1 second = 60 ticks
            ticks = GameLogic.getLogicTicRate()
            try:
                # Compute the speeds
                vx = speed / ticks
                rotation_speed = (speed / ticks) / 2.0
            # For the moment ignoring the division by zero
            # It happens apparently when the simulation starts
            except ZeroDivisionError:
                pass

            # Collision avoidance using the Blender radar sensor
            if self._radar_r['Rcollision']:
                rz = rotation_speed
            elif self._radar_l['Lcollision']:
                rz = - rotation_speed
            # Test if the orientation of the robot is within tolerance
            elif -self.angle_tolerance < angle_diff < self.angle_tolerance:
                rz = 0
            # If not, rotate the robot in the corresponding direction
            else:
                rz = rotation_speed * rotation_direction

        # Give the movement instructions directly to the parent
        # The second parameter specifies a "local" movement
        parent.blender_obj.applyMovement([vx, 0, 0], True)
        parent.blender_obj.applyRotation([0, 0, rz], True)
