*Thanks for taking the time to fill an issue with MORSE!*

[Describe here your issue]

Minimal builder file to reproduce the issue:

```python
from morse.builder import *

## Paste here a (minimal) Builder script that triggers the issue

robot = ...

...

# if your issue is not related to a specfic middleware,
# you can use the socket interface to ease the testing.
robot.add_default_interface('socket')


# if your issue is independant of the environment,
# you can use the default one (in fast mode).
env = Environment('sandbox', fastmode = True)
env.set_camera_location([-18.0, -6.7, 10.8])
env.set_camera_rotation([1.09, 0, -1.14])

```

[if necessary, also attach add the `.blend` file used]

---
- *MORSE version:* [add the output of 'morse --version']
- *Blender version:* [check the output of 'morse check']
- *Python version:* [check the output of 'morse check']
