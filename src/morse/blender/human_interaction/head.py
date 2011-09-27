from bge import logic

def set_body_position(co):
    """
    During grabbing the head moves. The 'Head_Empty' needs to follow.
    """
    ow = co.owner

    head = logic.getCurrentScene().objects['Head']
    human = logic.getCurrentScene().objects['POS_EMPTY']


    if human['Manipulate']:
        ow.worldPosition = head.worldPosition
