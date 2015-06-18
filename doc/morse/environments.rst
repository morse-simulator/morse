Predefined Environments
=======================

MORSE ships with a set of predefined indoor and outdoor 3D environments you can
use.

You can also use your own environments by simply specifying the full path of a
Blender file in the ``Environment`` object of your :doc:`Builder script
<user/builder>`.


.. widegallery:: environments

.. note::
    Include these robots in your simulation by using the identifier below the pictures::

        from morse.builder import *
        
        #...

        env = Environment('<path>/<identifier>')

    Click on the environments above to get the exact path for each of them.

