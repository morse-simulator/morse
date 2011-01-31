import GameLogic

def heal_victim(contr):
    """ Change the 'Severity' property of a nearby victim

    When the victim is fully healed, set its status as not Injured
    """
    h_key = contr.sensors['H_KEY']
    radar = contr.sensors['Radar']

    if h_key.triggered and h_key.positive:
        if radar.triggered and radar.positive:
            victim = radar.hitObject

            # Restore the health to the victim
            if victim['Severity'] > 0:
                victim['Severity'] = victim['Severity'] - 1
                # Set the colors depending on the severity of the injuries
                red = victim['Severity'] * 0.1
                green = 1 - red
                victim.color = [red, green, 0.0, 1.0]

            # Change the status
            if victim['Severity'] == 0:
                victim['Injured'] = False
