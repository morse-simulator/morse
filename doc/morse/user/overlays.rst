Component overlays
==================

A **component overlay** is a special *pseudo* component (sensor or actuator)
that overrides the default component behaviour to better fit into a 
particular architecture.

.. warning::
  As of `MORSE 0.6`, only :doc:`services <../dev/services>` can be overrided. In future
  versions, the *stream-oriented* interfaces of components (like ROS topics
  or YARP port) may be overridable as well.

Overlay example
---------------

The example below present a typical use for overlays: MORSE provides a
:doc:`pan-tilt unit <actuators/ptu>` actuator that offers a method,
:meth:`morse.actuators.ptu.PTU.set_pan_tilt`, to set the pan and 
tilt angles of the PTU.

But in your architecture, you are using 2 different methods, ``SetTilt`` and
``SetPan``.

The following overlay for maps your functions to MORSE default ones:

.. code-block:: python

    from morse.core.services import service
    from morse.core.overlay import MorseOverlay
    from morse.core import status

    class MyPTU(MorseOverlay):
        
        def __init__(self, overlaid_object):
            # Call the constructor of the parent class
            super(self.__class__,self).__init__(overlaid_object)
            
            self.last_tilt = 0.0
            self.last_pan = 0.0
        
        @service
        def SetTilt(self, tilt):
            
            self.last_tilt = float(tilt)
            self.overlaid_object.set_tilt_pan(self.last_tilt, self.last_pan)
        
        @service
        def SetPan(self, pan):
            
            self.last_pan = float(pan)
            self.overlaid_object.set_tilt_pan(self.last_tilt, self.last_pan)

You can save this overlay anywhere, for instance in a ``morse.my_overlays.py``
file, accessible from MORSE Python path.

For asynchronous service, it is a bit more complex, as we need to provide a 
callback. The :meth:`morse.core.overlay.MorseOverlay.chain_callback` takes care
about this operation. You can pass an optional function to this method to
modify the returned data, or modify the state of your object.

This new callback *must* take one parameter (a tuple ``(status,
result)``) and return a new tuple ``(status, result)``:

.. code-block:: python

    from morse.core.services import async_service
    from morse.core.overlay import MorseOverlay
    from morse.core import status

    class MyPTU(MorseOverlay):
        
        def __init__(self, overlaid_object):
            # Call the constructor of the parent class
            super(self.__class__,self).__init__(overlaid_object)
            
            self.last_tilt = 0.0
            self.last_pan = 0.0

        def format_pan_tilt_return(self, result):
            # this callback will be called when SetTilt or SetPan
            # completes.
            # It simply re-format the return value of the asynchronous
            # services.

            status, value = result

            return (status, 
                    "PTU->{:.2f},{:.2f}".format(self.last_pan, self.last_tilt))
        
        @async_service
        def SetTilt(self, tilt):
            self.last_tilt = float(tilt)
            self.overlaid_object.set_tilt_pan(
                    self.chain_callback(self.format_pan_tilt_return), \
                    self.last_tilt, self.last_pan)

        @async_service
        def SetPan(self, pan):
            self.last_pan = float(pan)
            self.overlaid_object.set_tilt_pan(
                    self.chain_callback(self.format_pan_tilt_return), \
                    self.last_tilt, self.last_pan)


.. warning::
    The behaviour is currently undefined in case of service name collision
    between the original sensor services and the services defined in the overlay.

Scene setup
-----------

With the MORSE Builder API
++++++++++++++++++++++++++

Components can be easily overlaid from the :doc:`MORSE Builder API
<../user/builder>` with the method
:meth:`morse.builder.abstractcomponent.AbstractComponent.add_overlay`.

This method takes two parameters, the middleware to use (cf
:mod:`morse.builder.data` for the list of available options) and the
full-qualified Python name of the overlay class (for instance,
``morse.my_overlays.MyPTU``)

The following example is taken from one of the ROS unit-tests:

.. code-block:: python

   #! /usr/bin/env morseexec

   from morse.builder import *

   robot = ATRV()
    
   waypoint = Waypoint()
   robot.append(waypoint)
    
   waypoint.add_overlay('ros', 'morse.middleware.ros.overlays.waypoints.WayPoint')
    
   env = Environment('indoors-1/indoor-1')


Here, the ``waypoint`` actuator get overlaid by the ``WayPoint`` class defined
in the module :mod:`morse.middleware.ros.overlays.waypoints`.

Name remapping
--------------

Overlays also allow to redefine the component name by overloading the 
:meth:`morse.core.abstractobject.AbstractObject.name` method.

Let's complete our previous example:

.. code-block:: python

    # [...]

    class MyPTU(MorseOverlay):
        
        # [...]
        
        def name():
            return "MyPTU"
        
        # [...]

In this case, at initialization, a new (pseudo) component (called ``MyPTU`` in 
this case) is created, with services as defined in the overlay class.

The original component is also created and remain available as usual.

