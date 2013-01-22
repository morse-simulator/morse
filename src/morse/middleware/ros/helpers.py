
import rospkg
import sys
import os

def ros_add_to_syspath(node):
    rp = rospkg.RosPack()
    path = os.path.join(rp.get_path(node),"src")
    sys.path.append(path)

