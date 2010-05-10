import sys, os

""" Add the MORSE script directories to the Python path used by Blender. """

try:
	ors_root = os.environ['ORS_ROOT']
except KeyError:
	print ("MORSE WARNING: ORS_ROOT environment variable not found. Set it to the correct path")
	ors_Root = '.'

# List of the directories where MORSE should look for python scripts
script_dirs = ['scripts', 'scripts/helpers', 'scripts/middleware']

# Add the directories to the Python path Used by Blender
for dir in script_dirs:
	dir_path = os.path.join(ors_root, dir)
	if dir_path not in sys.path:
		sys.path.append(dir_path)
	
print ("Welcome to the MORSE simulator")
print ("==============================")
print ("\nUsing path: {0}\n".format(sys.path))

# For backwards compatibility with the current modules using YARP
# TODO: This should be dynamic to allow users to choose a middleware
import init_middleware
