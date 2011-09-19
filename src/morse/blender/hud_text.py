def change_text(contr):
    """ Set the desired text in the HUD """
    obj = contr.owner
    obj['Text'] = '''
    * Esc key:        stop and exit the simulation
    * H key:           show this help message
    * W, S keys:    move main camera forward/bakwards
    * A, D keys:     move main camera left/right
    * R, F keys:     move main camera up/down
    * Left CTRL:    hold to aim main camera with mouse
    * Spacebar:     toggle cameras on and off
    * F9 key:         switch camera view
    * F11 key:       reset positions of all objects
    * F12 key:       emergency exit from the simulation
'''
