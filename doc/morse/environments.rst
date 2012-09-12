Predefined Environments
=======================

MORSE ships with a set of predefined indoor and outdoor environment you can use.

You can also use your own environments by simply specifying the full path of a
Blender file in the ``Environment`` object of your :doc:`Builder script
<user/builder>`.


.. widegallery:: environments

.. note::
    Include these robots in your simulation by using the identifier below the pictures::

        from morse.builder import *
        
        #...

        env = Environment('<path>/<identifier>')

    Check `the source
    <https://github.com/laas/morse/tree/master/data/environments>`_ to know the
    right path (*eg*, ``land-1/tree``).

