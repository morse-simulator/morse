from bge import logic

def set_body_position(co):
    """
    During grabbing the head moves. The 'Head_Empty' needs to follow.
    """
    ow = co.owner

    # get the suffix of the human to reference the right objects
    suffix = ow.name[-4:] if ow.name[-4] == "." else ""

    head = logic.getCurrentScene().objects['Head' + suffix]
    human = logic.getCurrentScene().objects['Human' + suffix]


    if human['Manipulate']:
        ow.worldPosition = head.worldPosition
