from bge import logic
from mathutils import Vector

def move(cont):
    """
    Move the hand to a nice position for carriing objects.
    This script is executed as long as the Property 'moveArm' is True
    """
    ow = cont.owner

    # get the suffix of the human to reference the right objects
    suffix = ow.name[-4:] if ow.name[-4] == "." else ""
    
    dest = logic.getCurrentScene().objects['IK_Pose_Empty.R' + suffix]
    hips = logic.getCurrentScene().objects['Hips_Empty' + suffix]
    left_hand = logic.getCurrentScene().objects['IK_Target_Empty.L' + suffix]
    human = logic.getCurrentScene().objects['Human' + suffix]

    
    # get the Vector to the right position
    if human['Manipulate']:
        vect = ow.getVectTo(dest)
    else:
        walk_hand_position = human.worldPosition + human.worldOrientation*Vector((0.3, -0.3, 0.9))
        vect = ow.getVectTo(walk_hand_position)
    # vect[0] is Distance
    # vect[1] and vect[2] are the Vector in global and local coordinates

    ow.applyMovement(vect[1]/50)
    # use global coordinates to move the right hand a bit to the destination
    hips.applyMovement([0.0, 0.0, (0.92 - hips.localPosition[2])/10])
    # move the hips down
    left_hand.applyMovement([0.0, 0.0, (0.9 - left_hand.localPosition[2])/10])
    # also move the left hand to prevent a unnatural pose

    if vect[0] < 0.02:
        # if the owner is near enough to the right position, set this position
        ow.worldPosition = dest.worldPosition if human['Manipulate'] else walk_hand_position
        ow['moveArm'] = False
        # stop this script from being executed all the time
        # interaction.py will set the property again if needed

