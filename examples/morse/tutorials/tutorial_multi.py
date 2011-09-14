import bpy

from morse.builder.morsebuilder import *

"""
This tutorial is dedicated to multi-node simulation management.
It contains three controlable robots whose simulation will be distributed on different nodes.
"""

### ATRV Robot ###
dala = Robot("atrv")
dala.name = "Dala"
dala_control = Actuator("v_omega")
dala_control.configure_mw("socket")
dala.append(dala_control)
dala_gyro = Sensor("gyroscope")
dala_gyro.configure_mw("socket")
dala.append(dala_gyro)
dala.location = [0,0,0]

### Jido Robot ###
jido = Robot("jido")
jido.name = "Jido"
jido_control = Actuator("v_omega")
jido_control.configure_mw("socket")
jido.append(jido_control)
jido_gyro = Sensor("gyroscope")
jido_gyro.configure_mw("socket")
jido.append(jido_gyro)
jido.location = [-4,0,0]

### ATRV Robot ###
mana = Robot("atrv")
mana.name = "Mana"
mana_control = Actuator("v_omega")
mana_control.configure_mw("socket")
mana.append(mana_control)
mana_gyro = Sensor("gyroscope")
mana_gyro.configure_mw("socket")
mana.append(mana_gyro)
mana.location = [0,-4,0]

### Environment ###
env = Environment('land-1/trees')
del env