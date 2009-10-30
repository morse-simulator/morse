import sys, os

try:
   scriptRoot = os.path.join(os.environ['ORS_ROOT'],'scripts')
except KeyError:
   scriptRoot = '.'

sys.path.append(scriptRoot)

import init_middleware

"""
# Startup yarp before even starting the Game Engine
import yarp
yarpNetwork = yarp.Network()

# Test to execute an external program before the GE
import test_onLoad
test_onLoad.gimme()
"""