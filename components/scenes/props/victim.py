import GameLogic

def heal(contr):
	""" Change the status of the victim
	
	Change the material to a green color,
	and the status to healed.
	"""

	obj = contr.owner

	obj['Injured'] = False
	obj['Gravity'] = 'small'
