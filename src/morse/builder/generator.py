import os
import bpy
import json

# XXX Hard-coded PATH, must be fixed

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
  http://www.blender.org/documentation/blender_python_api_2_57_release/bpy.types.BlendDataLibraries.html#bpy.types.BlendDataLibraries.load
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
    """ bpy.data.libraries.load(path) is 2.57 OK (but 2.56 NOK!)
    """
    objects = []
    with bpy.data.libraries.load(blend) as (src, _):
      objects = src.objects
    return objects
  def dump(self, dest):
    with open(dest, 'w') as fdict:
      json.dump(self.data, fdict, indent=1)
    # TODO pprint.pprint
  @property
  def data(self):
    return self._data

def test():
  import tempfile
  import morse.builder.data
  components = ComponentsData(morse.builder.data.MORSE_COMPONENTS)
  fd, path = tempfile.mkstemp(prefix="morse-components-", suffix=".py")
  try:
    components.dump(path)
  finally:
    os.close(fd)
  print("path: %s"%path)

"""
# write the data in a temp file
import sys
sys.path.append("/usr/local/lib/python3.1/dist-packages")
import morse.builder.generator
morse.builder.generator.test()
"""
