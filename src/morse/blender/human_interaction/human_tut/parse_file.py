from morse.core import blenderapi
import sys
import os

scene = blenderapi.scene()
scriptHolder = scene.objects['TUT_Script_Holder']

language = scriptHolder['Language']

# get the path once
for dir in sys.path:
    if os.path.exists(os.path.join(dir, "morse/blender/main.py")):
        path = os.path.join(dir, 'morse/blender/human_interaction/human_tut/data/' + language + '/')

def read_file():
    level = scriptHolder['Level']
    f = open(path + "explanations" + str(level) + ".txt", 'r')
    data=f.read()
    return(data)
