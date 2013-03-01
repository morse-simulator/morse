Contributing to MORSE
=====================

Get involved!
-------------

As an open-source project driven by the research community, your 
contributions are very welcome!

You can write to the list of developers of the project, writing to this 
address: `morse-dev@laas.fr <mailto:morse-dev@laas.fr>`_.

MORSE development can be tracked on the `MORSE GitHub repository
<https://github.com/laas/morse>`_::

    git clone git://github.com/laas/morse.git

If you want your feature to be merged in MORSE:

- First, `fork <https://help.github.com/articles/fork-a-repo>`_ ``laas/morse``
  on GitHub.
- Then, `send a pull request
  <https://help.github.com/articles/using-pull-requests>`_ with your changes.

You can also get the source directly from the LAAS master repository at:
``git://git.openrobots.org/git/robots/morse``

*We use GitHub as a mirror.*


Developers documentation
------------------------

Extending Morse
+++++++++++++++

.. toctree::
    :maxdepth: 1

    dev/file_hierarchy
    dev/coding_guidelines
    dev/adding_component
    dev/adding_robot
    dev/adding_datastream_handler
    dev/adding_modifier
    dev/services
    dev/new_middleware
    dev/testing

Morse internals
+++++++++++++++

.. toctree::
    :maxdepth: 1

    dev/component_object_model
    dev/entry_point
    dev/execution_loop
    dev/arguments_passing
    dev/time_event
    dev/services_internal

Code reference
--------------

Automatically generated code reference is `accessible here <code/morse.html>`_

Tips and how-to 
---------------

.. toctree::
    :glob:
    :maxdepth: 1

    user/tips/*
