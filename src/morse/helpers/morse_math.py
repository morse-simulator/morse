import logging; logger = logging.getLogger("morse." + __name__)
import math

def normalise_angle(angle):
    """ Force the given angle to be between PI and -PI
    
    This function expects an angle given in radians
    It will reduce the input angle to be less than PI,
    and give it the correct sign.

    Using new method proposed by David Hodo:
    hododav@tigermail.auburn.edu
    """
    return ((angle+math.pi)%(2*math.pi))-math.pi


def rotation_direction(current_angle, target_angle, tolerance, speed):
    """ Test the direction in which a rotation should be made

    Using the current angle of a component and the next desired angle.
    Angles are expected in radians """
    # Check which direction to rotate
    if current_angle < (target_angle - tolerance):
        rotation = speed
    elif current_angle > (target_angle + tolerance):
        rotation = -speed
    # If the angle is within the tolerance, don't rotate
    else:
        rotation = 0

    return rotation
