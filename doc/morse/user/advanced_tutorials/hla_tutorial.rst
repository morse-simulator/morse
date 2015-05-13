Basic interaction with another simulator using HLA : the Billard example :tag:`hla`
===================================================================================

In this tutorial, you will learn / test how to connect Morse with another
simulator engine, using HLA. We will use the default example simulator
provided by the CERTI implementation, a billard simulator.

.. warning::

    The tutorial assumes that you are familiar with HLA concept.

Setup
-----

First, verify that you have installed the needed Morse HLA stuff, by following 
the HLA section in the :doc:`installation notes <../installation/mw/hla>`.

Before running a distributed simulation using HLA/CERTI, it is necessary to
launch the RTIG (Run Time Interface Gateway) which will basically do the
synchronisation between the different simulators of the federation. So, open a
console and execute::

  $ rtig

We will now add our first instance to the federation, ``billard`` ,a poolroom
model provided by CERTI. So, in another console, start::

  $ billard -n foo -f Test -F Test.fed

where:

- **foo** is the name of the ambassador in the federation
- **test** is the name of the federation
- **Test.fed** is the name of the file describing the FOM of the federation.
  It is provided by CERTI.

You can press enter, and normally, you will see a red ball moving. Stop it now.

Reflecting attributes in Morse
------------------------------

We will now show how to define your scenario in order to communicate with this
federation.


First, we need to create one robot to reflect the ball in the poolroom.

.. warning::

    As, for the moment, there is no dynamic creation of the robot, robot name
    should be carefully chosen to match instances from other federates.


.. code-block:: python

    foo = Morsy()

The ``Test.fed`` shows basically two interesting attributes ``PositionX`` and
``PositionY``. In Morse, we will consider the :doc:`teleport actuator
<../actuators/teleport>` to reflect the position of an external robot.

.. code-block:: python

    foo = Morsy()
    
    teleport = Teleport()
    foo.append(teleport)
    

Now, we need to connect this stuff to the HLA world. We use the
``test_certi_intput`` which is written exactly for this specific purpose.

.. code-block:: python

    from morse.builder import *

    foo = Morsy()
    foo.scale = [10.0, 10.0, 1]

    teleport = Teleport()
    teleport.add_stream('hla', 'morse.middleware.hla.certi_test_input.CertiTestInput')
    foo.append(teleport)

    env = Environment('empty')
    env.configure_stream_manager(
            'hla', 
            fom = 'Test.fed', name = 'Morse', federation = 'Test', sync_point
            = 'Init', time_sync = True, timestep = 1.0)

    ground = bpymorse.get_object('Ground')
    ground.scale = [255.0, 55.0, 0.0065]
    ground.location = [250.0, 50.0, -0.06]
    env.set_camera_clip(0.1, 1000)
    env.set_camera_location([250, 50, 350])
    env.set_camera_rotation([0.0, 0.0, 0.0])
    env.set_camera_speed(10.0)

.. warning::

    The parameters in ``configure_stream_manager`` are really important, see
    :doc:`the hla middleware documentation <../middlewares/hla>` for a complete description.

.. note::

    You can play with the timestep value to see how it interacts with other
    simulators

.. note::

    The ``ground`` and ``env`` configuration here is not very important, but
    used to look like more a poolroom.

Now, start again the billard, and in another console, morse. Normally, Morse
should be blocked, waiting for the synchronisation point. Press enter in the
billard console, and you should see Morsy moving according the ball movement.

Exporting attributes from Morse
--------------------------------

Now, we will create another robot, and allow it to reflect its position in the
federation. For that, we will use a :doc:`pose sensor <../sensors/pose>` and a
keyboard to control it. 

.. code-block:: python

    bar = Morsy()
    bar.translate(x = 12, y = 12)

    kb = Keyboard()
    bar.append(kb)

    pose = Pose()
    bar.append(pose)
    pose.add_stream('hla', 'morse.middleware.hla.certi_test_output.CertiTestOutput')

If you start again the billard and Morse, you now must see a new black ball on
the billard. Moreover, if you move the robot in Morse with the keyboard, you
should see the black ball also moving in the billard.

At the end, your file must look like ``$MORSE_ROOT/share/morse/examples/tutorials/tutorial_hla.py``.
