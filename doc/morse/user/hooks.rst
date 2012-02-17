Component bindings with middlewares (hooks) 
===========================================

Description of hooks 
--------------------

MORSE sensors and actuators are completely independent of any middleware,
and do not include themselves any means of sharing their internal data.
To be able to use this data outside the simulator, it is necessary to bind
a middleware to each component, specifying the expected behaviour.

Every MORSE sensor class has a list called ``output_functions``, and every
actuator has a list called ``input_functions``. These lists are empty by
default, but during execution, any function listed in them will be called
each time the component executes its main task.

When a component is linked to a middleware, a function described in the 
middleware will be added to the corresponding list of functions (input or
output). The functions then become part of the component class, and can
read the data provided by the component via a *hook*, which is a data
structure common to all the components.

MORSE components have a dictionary called ``local_data`` that has the names
of the variables as keys, and stores the values of each variable. The values
stored in this dictionary can be of any type allowed in Python: String, Integer,
Float, List or Dictionary.

Data modifiers can also be applied to the ``local_data`` dictionary, to alter 
the data to fit the requirements of the simulation scenario.
In the case of sensors, the modifiers are applied before the data is sent
through the middleware. The opposite happens with actuators, which first read
data through middlewares and then change it using modifiers, before using it
inside of Blender.

A component can be connected to more than one middleware, and MORSE will keep
a communication channel for each middleware and export the data through all of
them when required.
Components can also use more than one modifier. These will change the data in
the same order as they are listed in the configuration (see below).

Component services are also configured in a way similar to middlewares and
modifiers, since they also make use of the ``local_data`` dictionary. However,
the internal workings of services are different, and are explained in the
developer documentation (:doc:`services <../dev/services>`).

Configuration 
-------------

This binding is currently done via a Python file called ``component_config.py``
which must be internal to the Blender scenario file. The configuration file 
should be edited from a **Text Editor** window in Blender.

It consists of three dictionaries indexed by the name of the components
(they are all optional):

- ``component_mw``: Lists which middlewares are bound to specific components.
  The value of the dictionary is either a list of strings (for only one
  middleware) or a list of lists (describing the binding to several middlewares).
  In both cases, each list describing a middleware consists of at least 2,
  strings, with the information detailed below:
  
  - The first element is the *fully qualified class name* of the Middleware, using
    the whole path to the corresponding Python script in MORSE. There are only
    a handful of options for this value at the moment:

    - ``morse.middleware.ros_mw.ROSClass``
    - ``morse.middleware.socket_mw.MorseSocketClass``
    - ``morse.middleware.yarp_mw.MorseYarpClass``
    - ``morse.middleware.pocolibs_mw.MorsePocolibsClass``
    - ``morse.middleware.text_mw.TextOutClass``

  - The second item is the name of the function to be added to the component's
    action list. To determine which function to use, refer to the documentation
    of the middleware.

  - The third item is optional. It is the path and filename (without file extension)
    of where the function listed in the second item is defined. This is necessary
    for non generic functions that can be added as extensions to the middleware.

  - The fourth item is optional. It is the name of the port that the middleware
    should use. In the case of sensors, it is the name of the poster that will be
    created. For actuators, it is the name that they will look for to connect.

- ``component_service``: Lists the middlewares that will take care of handling
  a service provided by a component. The value of the dictionary is a list of
  *fully qualified names* of Python classes inheriting from
  :py:class:`morse.core.request_manager.RequestManager` (like
  :py:class:`morse.middleware.socket_request_manager.SocketRequestManager`).

- ``component_modifier``: Lists the modifiers affecting each of the components. 
  The value of the dictionary is a list, where each element is itself a list 
  representing a modifier. Thus a single component can have multiple modifiers changing
  its data in sequence, following the order given here.
  Each modifier list has two elements: the name of the modifier and the name of the function to use.
  The format of these elements is the same as in ``component_mw``.

Example:

.. code-block:: python

    # Middleware binding for regular data exchange
    component_mw = {
      "Gyroscope": [["morse.middleware.yarp_mw.MorseYarpClass", "post_message"]],
      "GPS": [["morse.middleware.yarp_mw.MorseYarpClass", "post_message"], ["morse.middleware.socket_mw.MorseSocketClass", "post_message"]],
      "GPS.001": [["morse.middleware.yarp_mw.MorseYarpClass", "post_json_data", "morse/middleware/yarp/json_mod"]],
      "Motion_Controller": [["morse.middleware.yarp_mw.MorseYarpClass", "read_message"]],
      "Motion_Controller.001": [["morse.middleware.pocolibs_mw.MorsePocolibsClass", "read_genpos", "morse/middleware/pocolibs/actuators/genpos", "simu_locoSpeedRef"]],
      "CameraMain": [["morse.middleware.yarp_mw.MorseYarpClass", "post_image_RGBA"]]
    }

    # Component service binding with a middleware
    component_service = {
      "Motion_Controller": ["morse.middleware.yarp_request_manager.YarpRequestManager"],
    }
    
    # Modifier configuration
    component_modifier = {
      "GPS.001": [ ["morse.modifiers.ned.MorseNEDClass", "blender_to_ned"], ["morse.modifiers.utm.MorseUTMClass", "blender_to_utm"] ],
      "Motion_Controller": [ ["morse.modifiers.ned.MorseNEDClass", "ned_to_blender"] ]
    }


A fourth ``overlays`` dictionary may be added to specify overlays. See
:doc:`Component overlays <../user/overlays>` for details.
