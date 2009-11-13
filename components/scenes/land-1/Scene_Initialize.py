import sys, os

try:
   scriptRoot = os.path.join(os.environ['BLENDER_ROBOTICS_ROOT'],'scripts')
except KeyError:
   scriptRoot = '.'

try:
   libRoot = os.path.join(os.environ['BLENDER_ROBOTICS_ROOT'],'lib')
except KeyError:
   libRoot = '.'

sys.path.append(scriptRoot)
sys.path.append(libRoot)

from middleware.independent.IndependentBlender import *

print '######## SCENE INITIALIZATION ########'
print
print 'Scripts path: ', scriptRoot
print 'Lib path: ', libRoot	
print '\n'

#YARP initialization
if not hasattr(GameLogic, 'orsConnector'):
	GameLogic.orsConnector = MiddlewareConnector()
