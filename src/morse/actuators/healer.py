######################################################
#
#    healer.py        Blender 2.5x
#
#    A script to change the injured status of a
#    victim object. Used for the Rosace scenario
#
#
#    Gilberto Echeverria
#    28 / 01 / 2011
#
######################################################


import logging; logger = logging.getLogger("morse." + __name__)
import math
import GameLogic
import mathutils
import morse.core.actuator

class HealerActuatorClass(morse.core.actuator.MorseActuatorClass):
    """ Healer beam class

    This controller will receive an instruction to heal a nearby victim
    """

    def __init__(self, obj, parent=None):

        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(obj, parent)

        # Single command to heal
        # Receiving a value of 1 will activate the healing
        self.local_data['heal'] = 0

        logger.info('Component initialized')


    def default_action(self):
        """ Change the 'Severity' property of a nearby victim

        When the victim is fully healed, set its status as not Injured
        """
        contr = GameLogic.getCurrentController()
        radar = contr.sensors['Radar']

        if self.local_data['heal']:
            if radar.triggered and radar.positive:
                victim = radar.hitObject

                # Restore the health to the victim
                if victim['Severity'] > 0:
                    victim['Severity'] = victim['Severity'] - 1
                    # Set the colors depending on the severity of the injuries
                    red = 1 - victim['Severity'] * 0.05
                    green = 0.5 + red
                    victim.color = [red, green, 0.5, 1.0]

                # Change the status
                if victim['Severity'] == 0:
                    victim['Injured'] = False
                    # Reset the healing flag
                    self.local_data['heal'] = 0
