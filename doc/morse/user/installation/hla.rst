HLA
---

The High Level Architecture (HLA) is a standard framework that supports
simulations composed of different simulation components. Some introductory
courses about HLA are available `here <http://www.ecst.csuchico.edu/~hla/>`_.
A more complete description is also given in the :doc:`HLA Multinode Simulation <../../../user/multinode/hla>`.

Installing the CERTI
~~~~~~~~~~~~~~~~~~~~
The HLA implementation on which the multi-node version of MORSE is build is
the `CERTI <https://savannah.nongnu.org/projects/certi>`_. To install the CERTI,
you have to get the sources from the CERTI CVS repository::

$ cvs -z3 -d:pserver:anonymous@cvs.savannah.nongnu.org:/sources/certi checkout certi

Then::

$ cd certi && mkdir build && cd build && cmake .. && make install

Installing PyHLA
~~~~~~~~~~~~~~~~

You also have to create the corresponding Python binding in order to have
MORSE able to use the CERTI. 
The PyHLA sources are also available from the CERTI repository::

$ cvs -z3 -d:pserver:anonymous@cvs.savannah.nongnu.org:/sources/certi checkout applications/PyHLA pyHLA

Then, to install it::

$ cd pyHLA && mkdir build && cd build && cmake .. && make install

Depending on your system configuration, you may have to configure PyHLA to use
the Python 3.2 executable and libraries.

Then you will have to update your PYTHONPATH so that MORSE will find the PyHLA
components.
