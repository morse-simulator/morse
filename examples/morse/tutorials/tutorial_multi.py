import bpy

from morse.builder.morsebuilder import *

"""
This tutorial is dedicated to multi-node simulation management.
It contains three controlable robots whose simulation will be distributed on different nodes.
"""

### ATRV Robot ###
dala = Robot("atrv")
dala.name = "Dala"
dala_control = Controller("morse_vw_control")
dala.append(dala_control)
dala_gyro = Sensor("morse_gyroscope")
dala.append(dala_gyro)
dala.location = [0,0,0]

### Jido Robot ###
jido = Robot("jido")
jido.name = "Jido"
jido_control = Controller("morse_vw_control")
jido.append(jido_control)
jido_gyro = Sensor("morse_gyroscope")
jido.append(jido_gyro)
jido.location = [-4,0,0]

### ATRV Robot ###
mana = Robot("atrv")
mana.name = "Mana"
mana_control = Controller("morse_vw_control")
mana.append(mana_control)
mana_gyro = Sensor("morse_gyroscope")
mana.append(mana_gyro)
mana.location = [0,-4,0]

### Middlewares ###
sock = Middleware("socket_empty")
sock.configure(dala_control)
sock.configure(dala_gyro)
sock.configure(jido_control)
sock.configure(jido_gyro)
sock.configure(mana_control)
sock.configure(mana_gyro)
