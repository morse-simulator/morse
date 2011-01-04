import GameLogic
import math
import morse.helpers.actuator
import morse.helpers.math as morse_math

class PlatineActuatorClass(morse.helpers.actuator.MorseActuatorClass):
    """ Controller for pant tilt unit (platine)

    Reads 3 angles and applies them to the object and its children
    """

    def __init__(self, obj, parent=None):
        print ('######## PLATINE INITIALIZATION ########')
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(obj, parent)

        #self.local_data['roll'] = 0.0
        #self.local_data['pitch'] = 0.0
        #self.local_data['yaw'] = 0.0
        #self.data_keys = ['roll', 'pitch', 'yaw']

        self._speed = self.blender_obj['Speed']
        # Define the tolerance to the desired angle
        self._tolerance = math.radians(0.5)

        self.local_data['pan'] = 0.0
        self.local_data['tilt'] = 0.0
        
        print ('######## PLATINE INITIALIZED ########')



    def default_action(self):
        """ Apply rotation to the platine unit """

        # Reset movement variables
        rx, ry, rz = 0.0, 0.0, 0.0

        # Tick rate is the real measure of time in Blender.
        # By default it is set to 60, regardles of the FPS
        # If logic tick rate is 60, then: 1 second = 60 ticks
        ticks = GameLogic.getLogicTicRate()

        try:
            normal_speed = self._speed / ticks
        # For the moment ignoring the division by zero
        # It happens apparently when the simulation starts
        except ZeroDivisionError:
            pass

        current_pan = self.position_3d.yaw
        current_tilt = self.position_3d.pitch

        # Get the angles in a range of -PI, PI
        target_pan = morse_math.normalise_angle(self.local_data['pan'])
        target_tilt = morse_math.normalise_angle(self.local_data['tilt'])

        # Determine the direction of the rotation, if any
        ry = morse_math.rotation_direction(current_tilt, target_tilt, self._tolerance, normal_speed)
        rz = morse_math.rotation_direction(current_pan, target_pan, self._tolerance, normal_speed)

        # Give the rotation instructions directly to the parent
        # The second parameter specifies a "local" movement
        self.blender_obj.applyRotation([rx, ry, rz], True)
