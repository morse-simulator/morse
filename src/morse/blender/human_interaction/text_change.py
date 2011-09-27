from bge import logic



def set_text(contr):
    """
    Sets the text on the overlay scene according to recieved messages
    """
    ow = contr.owner
    human = logic.getSceneList()[0].objects['Hand_Grab.R']

    message = contr.sensors['Message']
    
    try:
        ow['Text'] = message.bodies[0]
        if human['selected'] != '' and human['selected'] !=  'None':
            ow['Text'] = ow['Text'].replace('Pick up', '')
    except IndexError:
        pass
