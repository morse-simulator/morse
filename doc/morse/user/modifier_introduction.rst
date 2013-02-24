Modifiers 
=========

Modifier processing is a specific phase between datastream processing and real
component processing. It allows to make some generic transformations on the data
(for example, convert them from one frame convention to another one) or to alter 
the ``qualities`` of data to make them more realistic. 

List of existing modifiers 
--------------------------

.. toctree::
    :glob:
    :maxdepth: 1

    modifiers/*

Adding a modifier to a component
--------------------------------

Binding a component to use a modifier is done through the :doc:`MORSE Builder
API <../../user/builder>`, using the method
:py:meth:`morse.builder.abstractcomponent.AbstractComponent.alter`. For
example, to transform coordinates from local Blender frame to some absolute
UTM reference frame, you can use the :doc:`UTM conversion modifier
<../../user/modifiers/utm>` in this way:

.. code-block:: python

    from morse.builder import *
             
    robot = ATRV()

    gps = GPS()
    gps.add_stream('socket')
    gps.alter('UTM')
    robot.append(gps)

    env = Environment('empty', fastmode = True)
    env.properties(UTMXOffset='123456789.0', UTMYOffset=-4242.0, UTMZOffset=421.0)

Sometimes, you need to pass some arguments to your modifier, for example to set
the standard deviation of your :doc:`gaussian noise
<../../user/modifiers/pose_noise>`. In this case, you can use the following
syntax:

.. code-block:: python

    from morse.builder import *
    import math

    robot = ATRV()

    pose = Pose()
    pose.add_stream('socket')
    pose.alter('Noise', pos_std  = 0.10, rot_std = math.radians(10))

    env = Environment('empty', fastmode = True)


Creating a new modifier 
-----------------------

Please refer to the developer documentation: :doc:`Creating a modifier <../dev/adding_modifier>`.
