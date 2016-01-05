def change_text(contr):
    """ Set the desired text in the HUD """
    obj = contr.owner
    obj['Text'] = '''
  * Esc :       stop and exit the simulation
  * H :         show this help message
  * V :         toggle display of robot camera on HUD
  * W, S :      move main camera forward/bakwards
  * A, D :      move main camera left/right
  * R, F :      move main camera up/down
  * Left CTRL:  hold to aim main camera with mouse
  * F7 :        move CameraFP to robots
  * F8 :        reset position of CameraFP
  * F9 :        switch camera view
  * F11 :       reset positions of all objects
  * F12 :       forced exit from the simulation
'''

# TODO use a monospace font for text, not Arial.
