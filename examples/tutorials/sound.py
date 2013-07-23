""" MORSE Sound example

You can control this actuator using any middleware, here using pymorse:

    from pymorse import Morse
    sim = Morse()
    sim.robot.sound.publish({"mode":"stop"})
    sim.robot.sound.publish({"mode":"play"})
    sim.robot.sound.publish({"mode":"pause"})
    sim.close()
"""
from morse.builder import *

robot = ATRV()

keyboard = Keyboard()
keyboard.properties(Speed=3)
robot.append(keyboard)

sound = Sound()
# TODO edit the path to an existing file on your system
sound.open("/path/to/file.mp3")
sound.add_stream('socket')
robot.append(sound)

env = Environment('outdoors')

