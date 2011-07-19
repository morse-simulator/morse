def change_text(contr):
    """ Set the desired text in the HUD """
    obj = contr.owner
    obj['Text'] = '''
    * Esc key:        stop and exit the simulation
    * W, S keys:    move main camera forward/bakwards
    * A, D keys:     move main camera left/right
    * R, F keys:     move main camera up/down
    * H key:           show this help message
    * Spacebar:     toggle cameras on and off
    * F9 key:         switch camera view
    * F11 key:       reset all objects positions
    * F12 key:       emergency exit from the simulation
'''