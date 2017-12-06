Coding guidelines
=================

Style
-----

Coding conventions in *MORSE* follow Python's :pep:`008`.
This means in particular:

- use spaces for indentation, not tabs (4 spaces per level of indentation)
- at most 80 characters per line
- class names should use CamelCase, variables and methods should be lower case,
  possibly with underscores
- variables starting with an underscore are private
- comment the code

Logging
-------

Morse uses the standard `Python logging framework
<http://docs.python.org/3.2/library/logging.html>`_. Please use this module in your own Morse code,
and don't use ``print()``! Using print is cumbersome on big projects!

To use logging in your module:

.. code-block:: python

    import logging; logger = logging.getLogger("morse." + __name__)

    #...
    #...
    # logger.info("...")
    # logger.debug("...")
    # ...etc

By default, *MORSE* uses the ``INFO`` logging level.  You can easily
change the logging level, e.g., to ``DEBUG``, in a specific module by adding a line like this:

.. code-block:: python

    logger.setLevel(logging.DEBUG)


Test
----

*MORSE* comes with a useful :doc:`unit-testing system <testing>`. When you submit changes or new
code, please add some tests to demonstrate how your work is supposed to be used.
Adding such tests helps us to maintain the quality of the project's code in the long-term.
