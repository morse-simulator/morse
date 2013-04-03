import pymorse
import time
   
with pymorse.Morse() as morse:
    while True:
        pose = morse.robot.pose.get()
        morse.ghost.teleport.publish(pose)
        time.sleep(.1)
