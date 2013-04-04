# -*- coding: utf-8 -*-

import logging; logger = logging.getLogger("morse." + __name__)
######################################################
#
#    search_and_rescue.py        Blender 2.5x
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

import morse.core.sensor
from morse.core import blenderapi
from morse.core.exceptions import MorseRPCInvokationError
from morse.core.services import service
from morse.core.services import async_service
from morse.core import status
from collections import OrderedDict
from morse.helpers.components import add_data, add_property


class SearchAndRescue(morse.core.sensor.Sensor):
    """ 
    This is a multi functional component specific for Search and Rescue
    scenario, where the robot must be able to aid human victims. The
    sensor is capable of detecting any victim located within a cone in
    front of the robot, with a range delimited in the properties of the
    Blender object. The output of the sensor is a list of the robots and
    their positions in the simulated world. This sensor works only with
    the human victim object.

    Additionally, the sensor provides a number of services related to
    the capabilities of the robot to help the nearest victim:

        - Report on the condition of a victim
        - Report the capabilities of the robot
        - Heal a victim (if the robot has compatible capabilities with
          the requirements of the victim)

    In the test scenarios, human victims are shown in red.  When a robot
    approaches, if it has the adequate capabilities, it will be able to
    help the victims. When issued the command, the sensor will gradually
    change the colour of the victim to green, and its status to healthy.
    It will detect if a victim is in front of the robot. When instructed
    to heal the victim, it will change the Game Properties of the object
    to reduce its injured value.
    """
    _name = "Search And Rescue sensor"
    _short_desc = "High level sensor for search and rescue scenario"

    add_data('victim_dict', {}, "dict", \
             'A list of entries for each victim detected inside the cone'
             ' of the sensor. Keys are victim name. The value is a'
             ' dictionnary containing the coordinate of the victim, its'
             ' requirements, and the severity of its injuries')

    add_property('_heal_range', 5.0, 'Heal_range', 'float',
                'maximum distance from which it is possible to heal a'
                ' victim. Even if the victim can be detected by the '
                'sensor, it can’t be healed unless its distance from the '
                'robot is less than this value.')

    add_property('_abilities', "", 'Abilities', 'string',
                 'represents a list of numbers, separated by comas, that '
                 'represent the equipment capabilities of the robot. This '
                 'information should be used by the operator of the robot '
                 'to determine if it is capable of helping a victim or not.')

    # These properties are not used directly in the logic, but are used
    # in the builder to create the radar properly.
    # These value cannot be changed dynamically in bge.
    add_property('_angle', 60.0, 'Angle', 'float',
                 'Aperture angle of the radar capable of detecting the '
                 ' victims (in degree)')
    add_property('_distance', 10.0, 'Distance', 'float',
                 'Detection distance in meter. Victims further '
                 'way from the gripper than this distance will not be '
                 'detected')
    add_property('_freq', 3.0, 'Freq', 'float',
                 'change the delay required to heal a victim. This '
                 'number is expressed as the number of tics that are '
                 'ignored before taking action. A lower number will '
                 'produce a lower delay')

    def __init__(self, obj, parent=None):

        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__, self).__init__(obj, parent)

        # Definition for number of tics required to heal a victim
        self._delay = 10

        # Flag to indicate when the component is trying to heal a victim
        self._healing = False
        self._heal_delay = self._delay
        self._capable_to_heal = False

        # Variables to store information about the victim nearest to the robot
        self._nearest_victim = None
        self._nearest_distance = 999999

        # Convert the 'Abilities' property from a string into a list of ints
        #obj['Abilities'] = obj['Abilities'].split(',')
        self._abilities = [int(x) for x in self._abilities.split(',')]

        self._detect_distance = 10.0

        logger.info("Component initialized, runs at %.2f Hz ", self.frequency)


    @async_service
    def heal(self):
        """
        Reduce the Severity value of the nearest victim.
        When the value reaches ‘0’, change the Injured status of the victim to False.
        The service terminates when the victim is fully healed.
        """
        self._healing = True

    @service
    def get_robot_abilities(self):
        """
        Returns the list describing  the abilities with which the robot
        is equipped.
        It must match the requirements of the victim for the robot to be
        able to heal it.
        """
        return self._abilities

    @service
    def get_victim_requirements(self):
        """
        Returns the list describing the type of injuries sustained by the victim
        """
        if self._nearest_victim:
            return self._nearest_victim['Requirements']
        else:
            message = "No victim within range (%.2f m)" % self._detect_distance
            raise MorseRPCInvokationError(message)

    @service
    def get_victim_severity(self):
        """
        Returns the integer indicating the victim healing priority
        """
        if self._nearest_victim:
            return self._nearest_victim['Severity']
        else:
            message = "No victim within range (%.2f m)" % self._detect_distance
            raise MorseRPCInvokationError(message)

    def _heal_victim(self):
        """ Change the 'Severity' property of a nearby victim

        When the victim is fully healed, set its status as not Injured
        """

        victim = self._nearest_victim
        logger.debug("Healing victim %s at distance %f" %
                                    (victim.name, self._nearest_distance))
        # Check that the victim is whithing the valid range and that
        #  the robot is equiped to help the victim
        if self._nearest_distance < self._heal_range:
            # Check the abilities of the robot
            for ability in self._abilities:
                if ability in victim['Requirements']:
                    self._capable_to_heal = True
                    break

            # Quit if the robot has not the required capabilities
            if not self._capable_to_heal:
                self._healing = False
                message = "Not equipped of helping victim"
                self.completed(status.FAILED, message)
                return

            # Delay some time to heal
            self._heal_delay -= 1

            # When the delay has finished, change the victim status
            if self._heal_delay == 0:
                for ability in self._abilities:
                    # Remove the needs of the victim when compatible with the
                    #  abilities of the robot
                    if ability in victim['Requirements']:
                        victim['Requirements'].remove(ability)

                # Reset the healing flags
                self._healing = False
                self._heal_delay = self._delay

                # Mark the victim as healed
                if victim['Requirements'] == []:
                    victim['Severity'] = 0
                    victim.color = [0.5, 1.0, 0.5, 1.0]
                    message = "Victim '%s' healed" % victim.name
                else:
                    victim.color = [0.5, 0.5, 1.0, 1.0]
                    message = "Victim '%s' partially healed" % victim.name

                self.completed(status.SUCCESS, message)

        else:
            self._healing = False
            self._heal_delay = self._delay
            message = "No victim within range (%.2f m)" % self._heal_range
            self.completed(status.FAILED, message)


    def default_action(self):
        """ Look for nearby victims, and heal when instructed to """
        # Look for victims in the cone of the sensor
        contr = blenderapi.controller()
        radar = contr.sensors['Radar']

        # Clear the variables for the victims
        self.local_data['victim_dict'] = {}
        self._nearest_victim = None
        self._nearest_distance = 999999
        self._detect_distance = radar.distance

        if radar.positive:
            logger.debug('radar positive')
            for victim_obj in radar.hitObjectList:
                if victim_obj.name != self.robot_parent.name():
                    victim_position = victim_obj.worldPosition
                    # Fill the data structure for the victim
                    victim_coordinate = OrderedDict([
                                    ('x', victim_position[0]),
                                    ('y', victim_position[1]),
                                    ('z', victim_position[2])   ])
                    victim_d = OrderedDict([
                                  ('coordinate', victim_coordinate),
                                  ('requirements', victim_obj['Requirements']),
                                  ('severity', victim_obj['Severity'])    ])
                    self.local_data['victim_dict'][victim_obj.name] = victim_d

                    # Find the closest victim and its distance
                    new_distance = self.bge_object.getDistanceTo(victim_obj)
                    if new_distance < self._nearest_distance:
                        self._nearest_victim = victim_obj
                        self._nearest_distance = new_distance

            # When instructed to do so, help a victim
            if self._healing:
                self._heal_victim()
        else:
            if self._healing:
                self._healing = False
                message = "No victim within range (%.2f m)" % \
                                    self._detect_distance
                self.completed(status.FAILED, message)
