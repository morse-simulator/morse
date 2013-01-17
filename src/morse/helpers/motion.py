import math

def object_is_moving(bge_object):
    """ Check if an object is currently in motion
        
    Will work only for 'dynamic' or 'rigid body' objects in Blender.
    An object is considered to be in motion if any of the three
    components of its velocity vector is greater than an arbitrary
    tolerance.
    """
    # Minimum speed that will trigger the motion "sensor"
    motion_tolerance = 0.01
    speed = bge_object.getLinearVelocity()
    moving = False
    for i in range(3):
        # Test that the local speed is larger than a predefined limit
        if math.fabs(speed[i]) > motion_tolerance:
            moving = True

    return moving
