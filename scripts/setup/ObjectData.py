import GameLogic
import re
import json

def get_object_data(contr):
	""" Function to retrieve an object's name, parent and associated port."""
	# Get the game object this controller is on:
	ob = contr.owner
	parent = ob.parent

	# If there is no parent (when testing individual component)
	#  set this component as its own parent
	if not parent:
		parent = ob

	port_name = 'robots/{0}/{1}'.format(parent.name, ob['Component_Type'])

	return (ob, parent, port_name)


def get_robot_data(contr):
	""" Function to retrieve a robot's name and associated port."""
	#Get the name of the object (robot)
	ob = contr.owner

	port_name = 'robots/{0}'.format(ob.name)

	return (ob, port_name)


def decode_message(json_data):
    """ Decode a data structure using JSON.
        The data is initially a string.
        Returns a Python object,
        either an int, double, string, list or dictionary."""
    # Remove the quotations at the start and end of the string
    json_data = re.sub(r'"(.*)"', r'\1', json_data)
    # Unescape all other quotation marks
    json_data = re.sub(r'\\(.)', r'\1', json_data)
    clean_data = json.loads(json_data)

    return clean_data
