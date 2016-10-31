Coding guidelines
=================

Style
-----

Coding conventions in *MORSE* follow Python's :pep:`008`.
This means in particular:

- use spaces for indentation, not tabs (4 spaces per level of indent)
- at most 80 characters per line
- class names using CamelCase, variables and method names lower case,
  possibly with underscores.
- variable names starting with an underscore are private
- comment the code.

Logging
-------

MORSE uses the standard `Python logging framework
<http://docs.python.org/3.2/library/logging.html>`_. Please use logging
rather than ``print()``! Using print is inconvenient in big projects!

To use logging in your module:

.. code-block:: python

    import logging; logger = logging.getLogger("morse." + __name__)

    #...
    #...
    # logger.info("...")
    # logger.debug("...")
    # ...etc

By default *MORSE* is set to use the ``INFO`` logging level.  You can easily
set the logging level to, say, ``DEBUG``, in a specific module by adding
this line to it:

.. code-block:: python

    logger.setLevel(logging.DEBUG)


Test
----

*MORSE* has a :doc:`unit-testing system <testing>`. When you submit
additions or changes, it is helpful if you add some tests which
demonstrate how your work is supposed to be used. And adding tests also
helps the maintenance of the project in the long-term.
