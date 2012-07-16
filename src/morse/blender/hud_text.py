def change_text(contr):
    """ Set the desired text in the HUD """
    obj = contr.owner
    obj['Text'] = '''
    * Esc key:        stop and exit the simulation
    * H key:           show this help message
    * V key:           toggle display of robot camera on HUD
    * W, S keys:    move main camera forward/bakwards
    * A, D keys:     move main camera left/right
    * R, F keys:     move main camera up/down
    * Left CTRL:    hold to aim main camera with mouse
    * F5 key:         toggle camera/human movement
    * F8 key:         reset position of CameraFP
    * F9 key:         switch camera view
    * F11 key:       reset positions of all objects
    * F12 key:       forced exit from the simulation
'''
