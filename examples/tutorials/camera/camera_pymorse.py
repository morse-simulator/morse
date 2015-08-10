import base64
from pymorse import Morse

with Morse() as sim:
    data = sim.r.v.get()

width = data['width']
height = data['height']
# data['image'] is RGBA base64 encoded
buff = base64.b64decode( data['image'] )

# using scipy and numpy
import numpy
import scipy.misc
image = numpy.ndarray(shape=(height, width, 4), buffer=buff, dtype='uint8')
scipy.misc.imsave('scipy.png', image)

# using Python Imaging Library (PIL)
from PIL import Image
image = Image.frombuffer('RGBA', (width, height), buff, 'raw', 'RGBA', 0, 1)
image.save('pil.png')
