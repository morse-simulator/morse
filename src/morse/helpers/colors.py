from Blender import Material, Object

#Little helper fonction to calculate the hue of a color
def RGBtoHue(rgbList):
	R = rgbList[0]
	G = rgbList[1]
	B = rgbList[2]
	
	# min, max, delta;
	min_rgb = min( R, G, B )
	max_rgb = max( R, G, B )
	delta = max_rgb - min_rgb

	if not delta:
		return 0
	if R == max_rgb:
		H = ( G - B ) / delta # between yellow & magenta
	elif G == max_rgb:
		H = 2 + ( B - R ) / delta # between cyan & yellow
	else:
		H = 4 + ( R - G ) / delta # between magenta & cyan
	H *= 60 # convert to deg
	if H < 0:
		H += 360
	return int(H)

#this method retrieve the first material of the first mesh of an object
#and return the hue of this material.
def retrieveHue(obj):
	mesh = obj.meshes[0] # There can be more than one mesh...
	if mesh != None:
		bMat = Material.Get(mesh.getMaterialName(0)[2:])
		return RGBtoHue(bMat.getRGBCol())
	
	return None
