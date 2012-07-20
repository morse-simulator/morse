Unit-testing in MORSE
=====================

Introduction
------------

Unit-tests (using the standard Python unit-testing framework) can be added
to MORSE by creating a test class inheriting from 
:py:class:`morse.testing.testing.MorseTestCase`.

We contributing code to MORSE, we recommend you to create unit-tests for the new
features in subdirectories of `$MORSE_ROOT/testing`.

Once complete, you can add your unit-test to `$MORSE_ROOT/testing/test_all.py`.

Writing tests
-------------

The MorseTestCase
+++++++++++++++++

Compared to the standard `TestCase <http://docs.python.org/library/unittest.html#unittest.TestCase>`_
class, the :py:class:`morse.testing.testing.MorseTestCase` takes care of starting/closing
MORSE and initializing it with a specified environment.

This environment is defined by overloading the :py:meth:`morse.testing.testing.MorseTestCase.setUpEnv`
with a description of the environment using the :doc:`Builder API <../user/builder>`.
Moreover, you can overload the
:py:meth:`morse.testing.testing.MorseTestCase.setUpMw` and
:py:meth:`morse.testing.testing.MorseTestCase.tearDownMw` if you want to
create some specific environment before launching the Morse simulator, and to
clean it

Complete example
++++++++++++++++

This example create a new scenario with two robots in an indoor environment, and then
checks that MORSE `list_robots` control service actually returns both robots.

.. code-block:: python

    from morse.testing.testing import MorseTestCase

    class BaseTest(MorseTestCase):
    
        def setUpEnv(self):
            """ Defines the test scenario, using the Builder API.
            """
            
            # Adding 2 robots
            robot1 = Robot('jido')        
            robot2 = Robot('atrv')
            
            env = Environment('indoors-1/indoor-1')
    
        def test_list_robots(self):
            """ This test is guaranteed to be started only when the simulator
            is ready.
            """
            
            # Initialize a socket connection to the simulator
            import socket
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect(("localhost", 4000))
            sockf = s.makefile()
            
            # Queries for the list of robots
            s.send("id1 simulation list_robots\n")
            
            result = sockf.readline()
            id, success, robots = result.strip().split(' ', 2)
            self.assertEquals(success, "SUCCESS")
            
            import ast
            robotsset = set(ast.literal_eval(robots))
            self.assertEquals(robotsset, {'Jido', 'ATRV'})
            sockf.close()
            s.close()



Compiling MORSE to allow testing
--------------------------------

To be able to run the test suite, you must have compiled MORSE with the option 
``PYMORSE_SUPPORT`` enabled. For example::

  $ cmake -DPYMORSE_SUPPORT=ON [other options] ..

After doing the initial install, you'll need to recompile. For example::

  $ make install
  $ make rebuild_cache

After the installation is complete, you will now be able to run::

  $ make test

This will start launching MORSE with a series of unit tests, to evaluate that
the creation of scenes and some of the components are running properly on your
system.


    
Running tests
-------------

Running all MORSE tests
+++++++++++++++++++++++

The MORSE tests infrastructure is integrated with cmake, so you can run `make
test` to check that all currently defined unit-tests for MORSE pass. 

ROS tests
+++++++++

To run ROS tests, you need first to set the variable `MORSE_SRC_ROOT` to the
root of your MORSE source. Moreover, you need to generate messages for the
morsetesting helper: to do so, you must go in
`${MORSE_SRC_ROOT}/testing/middlewares/ros/morsetesting` and then use rosmake
to build them.

Tests log
+++++++++

The complete log of a test is available in the `${testName}.log` file, created
in the current directory, where testName is the name of your class test (in
the previous example, it will create the `BaseTest.log` file.

Running a test as a standalone application
++++++++++++++++++++++++++++++++++++++++++

We can run tests case by invoking:

.. code-block:: python

  MorseTestRunner().run(tests)

It is convenient to add at the end of a test-case the following lines:

.. code-block:: python

     if __name__ == "__main__":
        import unittest
        from morse.testing.testing import MorseTestRunner
        suite = unittest.TestLoader().loadTestsFromTestCase(<Your test class>)
        sys.exit(not MorseTestRunner().run(suite).wasSuccessful())

Thus, you can run your test by simply call it with the Python VM.
