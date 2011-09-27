from bge import logic

def toggle_grabbed(co):
    """
    Show a open or closed hand
    """
    message = co.sensors['Message']


    try:
        if message.bodies[-1] == '' or message.bodies[-1] == 'None':
            co.owner.replaceMesh('grabable', True, False)
        else:
             co.owner.replaceMesh('grabbed', True, False)
    except IndexError:
        pass

