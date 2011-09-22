from bge import logic

ow = logic.getCurrentController().owner
dest = logic.getCurrentScene().objects['IK_Pose_Empty.R']
hips = logic.getCurrentScene().objects['Hips_Empty']
left_hand = logic.getCurrentScene().objects['IK_Target_Empty.L']


def move():     # move the hand to a nice position for carriing objects
    # get the Vector to the right position
    vect = ow.getVectTo(dest)
    # vect[0] is Distance
    # vect[1] and vect[2] are the Vector in global and local coordinates
    
    ow.applyMovement(vect[1]/50)    # use global coordinates
    hips.applyMovement([0.0, 0.0, (0.92 - hips.localPosition[2])/10])
    left_hand.applyMovement([0.0, 0.0, (0.9 - left_hand.localPosition[2])/10])
    
    if vect[0] < 0.02:          # if the owner is near enough to the right position, set this position 
        ow.worldPosition = dest.worldPosition
        ow['moveArm'] = False       # stop this script from being executed all the time (interaction.py will set the property again if needed)

