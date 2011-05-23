import os
import bpy
import json

MORSE_COMPONENTS = '/usr/local/share/data/morse/components'

"""
components-dictionary-convention:
{
  'component-directory': {
    '.blend-file': ['main-object-name', 'child1-name', ...]
  }
}
"""

class ComponentsData(object):
  """ Build the components-dictionary (*/*.blend/Object/*)
  http://www.blender.org/documentation/250PythonDoc/bpy.types.BlendDataLibraries.html#bpy.types.BlendDataLibraries.load
  """
  def __init__(self, path):
    self.path = path
    self._data = {}
    self._update()
  def _update(self):
    for category in os.listdir(self.path):
      pathc = os.path.join(self.path, category)
      if os.path.isdir(pathc):
        self._data[category] = {}
        for blend in os.listdir(pathc):
          pathb = os.path.join(pathc, blend)
          if os.path.isfile(pathb) & blend.endswith('.blend'):
            self._data[category][blend[:-6]] = self.objects(pathb)
  def objects(self, blend):
    """ The problem is now that we don't respect the dictionary-convention, 
    which is: ['main-object-name', 'child1-name', ...] 
    (in order to select the main object in Component class) 

    bpy.data.libraries.load(path) is 2.57 OK , but 2.56 NOK!
    """
    objects = []
    with bpy.data.libraries.load(blend) as (src, dest):
      objects = src.objects
    return objects
  def dump(self, dest):
    fdict = open(dest, 'w')
    json.dump(self.data, fdict, indent=1)
    fdict.close()
  @property
  def data(self):
    return self._data

#components = ComponentsData(MORSE_COMPONENTS)
#components.dump('/tmp/morse-components.py')

