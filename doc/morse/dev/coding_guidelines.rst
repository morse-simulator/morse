Coding guidelines
=================

Style
-----

Coding conventions in *MORSE* follows Python's :pep:`008`.
It means in particular:

- use space for indentation, not tab (4 space)
- 80 character max by lines
- class name are in CamelCase, variables and methods are in lower case,
  possibly with underscores.
- variable starting by an underscore are private
- comment the code.

Logging
-------

Moreover, we rely on the standard `Python logging framework
<http://docs.python.org/3.2/library/logging.html>`_. Please use it, and don't
use ``print()``! Using print is cumbersome on big projects!

To use logging in your module:

.. code-block:: python

    import logging; logger = logging.getLogger("morse." + __name__)

    #...
    #...
    # logger.info("...")
    # logger.debug("...")
    # ...etc

*MORSE* is set to use by default the ``INFO`` logging level.  You can easily
set the logging level to ``DEBUG`` in a specific module by adding to it:

.. code-block:: python

    logger.setLevel(logging.DEBUG)


Test
----

*MORSE* have a cool :doc:`unit-testing system <testing>`. When you submit new
stuff, it is nice to add some tests which demonstrate how it is supposed to
work. It helps the maintenance of the project on the long-term.
