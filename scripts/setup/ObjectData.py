import GameLogic

def get_object_data(contr):
	# Get the game object this controller is on:
	ob = contr.owner
	parent = ob.parent

	# If there is no parent (when testing individual component)
	#  set this component as its own parent
	if not parent:
		parent = ob

	port_name = 'robots/{0}/{1}'.format(parent.name, ob['Component_Type'])

	return (ob, parent, port_name)
