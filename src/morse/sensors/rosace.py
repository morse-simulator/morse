import logging; logger = logging.getLogger("morse." + __name__)
######################################################
#
#    rosace.py        Blender 2.5x
#
#    Multiple sensor capable of detectin victims
#    and helping them, according to the requirements
#    and capabilities of the robot.
#    Used for the Rosace scenario
#
#
#    Gilberto Echeverria
#    16 / 06 / 2011
#
######################################################


import math
import GameLogic
import mathutils
import morse.core.sensor
from morse.core.exceptions import MorseRPCInvokationError
from morse.core.services import service
from morse.core.services import async_service
from morse.core import status


class RosaceSensorClass(morse.core.sensor.MorseSensorClass):
    """ Multi function sensor/actuator for Rosace scenario

    This sensor will be able to detect victims within a certain
    distance of the robot. Additionally, it provides the following services:
    - Report on the condition of a victim
    - Report the capabilities of the robot
    - Heal a victim (if compatible capabilities and requirements)
    """

    def __init__(self, obj, parent=None):

        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(obj, parent)

        # Flag to indicate when the component is trying to heal a victim
        self._healing = False

        # Variables to store information about the victim nearest to the robot
        self._nearest_victim = None
        self._nearest_distance = 999999

        # List of victims visible from this sensor
        self.local_data['victim_dict'] = {}

        # Convert the 'Abilities' property from a string into a list of ints
        #obj['Abilities'] = obj['Abilities'].split(',')
        obj['Abilities'] = [int(x) for x in obj['Abilities'].split(",")]


        logger.info('Component initialized')



    @async_service
    def heal(self):
        self._healing = True

    @service
    def get_robot_abilities(self):
        return self.blender_obj['Abilities']

    @service
    def get_victim_requirements(self):
        if self._nearest_victim:
            return self._nearest_victim['Requirements']
        else:
            message = "No victim within range (%.2f m)" % self.blender_obj['Heal_range']
            raise MorseRPCInvokationError(message)

    def _heal_victim(self):
        """ Change the 'Severity' property of a nearby victim

        When the victim is fully healed, set its status as not Injured
        """
        victim = self._nearest_victim
        logger.debug("Healing victim %s at distance %d" % (victim.name, self._nearest_distance))
        # Check that the victim is whithing the valid range and that
        #  the robot is equiped to help the victim
        if self._nearest_distance < self.blender_obj['Heal_range']:
            # Check the abilities of the robot
            for ability in self.blender_obj['Abilities']:
                # Remove the needs of the victim when compatible with the
                #  abilities of the robot
                if ability in victim['Requirements']:
                    victim['Requirements'].remove(ability)

            if victim['Requirements'] == []:
                # Reset the healing flag
                self._healing = False
                victim['Severity'] = 0
                victim.color = [0.5, 1.0, 0.5, 1.0]
                message = "Victim '%s' healed" % victim.name
            else:
                self._healing = False
                victim.color = [0.5, 0.5, 1.0, 1.0]
                message = "Victim '%s' partially healed" % victim.name

            self.completed(status.SUCCESS, message)

        else:
            self._healing = False
            message = "No victim within range (%.2f m)" % self.blender_obj['Heal_range']
            self.completed(status.FAILED, message)


    def default_action(self):
        """ Look for nearby victims, and heal when instructed to """
        # Look for victims in the cone of the sensor
        contr = GameLogic.getCurrentController()
        radar = contr.sensors['Radar']
        if radar.triggered and radar.positive:
            for victim_obj in radar.hitObjectList:
                victim_position = victim_obj.worldPosition
                self.local_data['victim_dict'][victim_obj.name] = [ [victim_position[0], victim_position[1], victim_position[2]], victim_obj['Requirements'] ]

                # Find the closest victim and its distance
                new_distance = self.blender_obj.getDistanceTo(victim_obj)
                if new_distance < self._nearest_distance:
                    self._nearest_victim = victim_obj
                    self._nearest_distance = new_distance

            # When instructed to do so, help a victim
            if self._healing:
                self._heal_victim()

        if radar.triggered and not radar.positive:
            # Clear the variables for the victims
            self.local_data['victim_dict'] = {}
            self._nearest_victim = None
            self._nearest_distance = 999999
