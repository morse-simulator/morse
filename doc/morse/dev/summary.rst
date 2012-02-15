Developers documentation
========================


Get involved!
-------------

You can write to the list of developers of the project, writing to this 
address: `morse-dev@laas.fr <mailto:morse-dev@laas.fr>`_.

MORSE developement can be tracked online with GitHub: `MORSE on GitHub 
<https://github.com/laas/morse>`_

Fork it from your own GitHub account, or get the source directly from 
the LAAS master repository::

    $ git clone git://git.openrobots.org/git/robots/morse

Coding guidelines
-----------------

Coding conventions in *MORSE* follows Python's :pep:`008`.

In particular, we use the standard Python logging framework. So please,
no ``print()``!

To use logging in your module:

.. code-block:: python

    import logging; logger = logging.getLogger("morse." + __name__)

    #...
    #...
    # logger.info("...")
    # logger.debug("...")
    # ...etc

*MORSE* is set to use by default the ``INFO`` logging level.
You can easily set the logging level to ``DEBUG`` in a specific module
by adding to it:

.. code-block:: python

    logger.setLevel(logging.DEBUG)

Overview 
--------
An overview of code organization in MORSE

:doc:`Overview for MORSE organization <dev_overview>`

Workflow
--------
Principle of interaction with Blender and the game engine

:doc:`Overview for MORSE workflow for developers <dev_workflow>`

Adding validation tests
-----------------------

New features are expected to provide unit-tests to validate them.

:doc:`Unit-testing in MORSE <testing>`

Creating a new component (robot, sensor or actuator)
----------------------------------------------------
Create a sensor. Explain the logic

:doc:`Tutorial: Adding a new component <adding_component>`

Creating a modifier
-------------------

EZ if in python, a bit more complex if we call an external library (cf impact on the dependencies) 

:doc:`Tutorial: Creating a modifier <adding_modifier>`

Adding the support of a new middleware
--------------------------------------
Introduce how to add the support for a new middleware

:doc:`Tutorial: Adding the support for a new middleware <new_middleware>`


Simulation supervision
----------------------

Besides component-specific services and data stream (documented on each component's
own documentation page), MORSE provides a set of *supervision services* that
may be used to remotely control the global behaviour of the simulator:

.. toctree::
	:maxdepth: 1

	../user/supervision_services
