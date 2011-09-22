Component overlays
==================

A **component overlay** is a special *pseudo* component (sensor or actuator)
that overrides the default component behaviour to better fit into a 
particular architecture.

.. warning::
  As of `MORSE 0.4`, only :doc:`services <../dev/services>` can be overrided. In future
  versions, the *stream-oriented* interfaces of components (like ROS topics
  or YARP port) may be overridable as well.

Overlay example
---------------

The example below present a typical use for overlays: MORSE provides a
:doc:`pan-tilt unit <actuators/ptu>` actuator that offers a method,
:py:meth:`morse.actuators.platine.Platine.set_pan_tilt`, to set the pan and 
tilt angles of the PTU.

But in your architecture, you are using 2 different methods, ``SetTilt`` and
``SetPan``.

The following overlay for the ``PlatineActuatorClass`` maps your functions 
to MORSE default ones:

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
            
            self.overlaid_object.set_tilt_pan(tilt, self.last_pan)
            self.last_tilt = float(tilt)
        
        @service
        def SetPan(self, pan):
            
            self.overlaid_object.set_tilt_pan(self.last_tilt, pan)
            self.last_pan = float(pan)

You can save this overlay anywhere, for instance in a ``morse.my_overlays.py``
file, accessible from MORSE Python path.

For asynchronous service, it is a bit more complex, as we need to provide a 
callback. The :py:meth:`morse.core.MorseOverlay.chain_callback` takes care
about this operation. You can pass an optional function to this method to
modify the returned data, or modify the state of your object.

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

        def log(self):
            print("Chaining callback")
        
        @async_service
        def SetTilt(self, tilt):
            self.overlaid_object.set_tilt_pan(self.chain_callback(), \
			tilt, self.last_pan)

        @async_service
        def SetPan(self, pan):
            self.overlaid_object.set_tilt_pan(self.chain_callback(log), \
			self.last_tilt, pan)

Scene setup
-----------

Overlays definitions must be added to the scene ``component_config.py`` (cf 
:doc:`../user/hooks` for details on the scene configuration file).

An overlay is defined by the triple (``middleware|request manager``, ``class of 
object to overlay``, ``class of the overlay``)

The following example shows how all instances of ``PlatineActuatorClass`` can be
overloaded with our ``MyPTU`` overlay, for the ``YarpRequestManager`` service manager:

.. code-block:: python

    overlays = {
      "YarpRequestManager": {
            "PlatineActuatorClass": "morse.my_overlays.MyPTU"
       }
    }

At initialization, MORSE will look for all components of type 
``PlatineActuatorClass``, and for each of them, it adds all services defined
above in ``MyPTU`` class.

.. warning::
    The behaviour is currently undefined in case of service name collision
    between the original sensor service and the services defined in the overlay.

Name remapping
--------------

Overlays also allow to redefine the component name by overloading the 
:py:meth:`morse.core.abstractobject.AbstractObject.name` method.

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

The original components are also created and stay available as usual.

.. warning::
    If an overlay overlays more than one object (for instance, two components of
    type ``PlatineActuatorClass`` have been added in the simulation), a name 
    conflict will arise (since two components will be created with the same 
    remapped name). In this case, the behaviour is currently undefined.


