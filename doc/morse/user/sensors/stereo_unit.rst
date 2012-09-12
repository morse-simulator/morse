Stereo Camera unit
==================

The purpose of this component is to link together one or more cameras, and
provide them with the possibility to move together as a single unit.  It will
also provide the connection interface to use the information of the cameras
attached to it. In the case of two cameras, it will provide the stereo
information generated from the two camera images.

Files
-----
- Blender: ``$MORSE_ROOT/data/sensors/stereo_unit.blend``
- Python: ``$MORSE_ROOT/src/morse/sensors/stereo_unit.py``

Applicable modifiers
--------------------

The modifiers to the images should be done by configuring the individual
cameras attached to this component.

Services
--------

- **capture**: (asynchronous service) The service takes an integer an argument
  and dispatch the call to all its individual cameras. The service ends when
  each camera has terminated its work.

Related components
------------------

A stereo unit needs to be the parent of one or more :doc:`cameras <video_camera>`.
Otherwise, it does no useful function.

The movement of the stereo unit is implemented by making it the child of a
:doc:`Pan-Tilt unit <../actuators/ptu>` actuator.

Here is an example of how to construct the whole stereo system to mount on top
of a robot, using the Builder API. Note the order in which components are
appended to each other, as this is important to get the desired functionality:

.. code-block:: python

    from morse.builder import *

    # Add a robot
    ATRV = Robot('atrv')
    ATRV.translate(z=0.1000)
    
    # A pan-tilt unit to be able to orient the cameras
    Platine = Actuator('ptu')
    Platine.translate(x=0.2000, z=0.9000)
    ATRV.append(Platine)
    
    # The STEREO UNIT, where the two cameras will be fixed
    Stereo = Sensor('stereo_unit')
    Stereo.translate(z=0.0400)
    Platine.append(Stereo)
    
    # Left camera
    CameraL = Sensor('video_camera')
    CameraL.translate(x=0.1000, y=0.2000, z=0.0700)
    Stereo.append(CameraL)
    CameraL.properties(capturing = True)
    CameraL.properties(cam_width = 320)
    CameraL.properties(cam_height = 240)
    CameraL.properties(cam_focal = 25.0000)
    
    # Right camera
    CameraR = Sensor('video_camera')
    CameraR.translate(x=0.1000, y=-0.2000, z=0.0700)
    Stereo.append(CameraR)
    CameraR.properties(capturing = True)
    CameraR.properties(cam_width = 320)
    CameraR.properties(cam_height = 240)
    CameraR.properties(cam_focal = 25.0000)
