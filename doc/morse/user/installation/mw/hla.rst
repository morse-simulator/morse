HLA :tag:`hla`
--------------

The High Level Architecture (HLA) is a standard framework that supports
simulations composed of different simulation components. Some introductory
courses about HLA are available `here <http://www.ecst.csuchico.edu/~hla/>`_.
A more complete description is also given in the :doc:`HLA Multinode Simulation <../../../user/multinode/hla>`.

To be able to use HLA integration in Morse, you need to install CERTI and its
python binding called pyHLA. There is several ways to install them, manually
or using a package manager such as **robotpkg**.

Manually
++++++++

CERTI is available on http://download.savannah.gnu.org/releases/certi/. The
version 3.4.3 is known to work properly with Morse. PyHLA can be found in
http://download.savannah.gnu.org/releases/certi/contrib/PyHLA/. The version
1.1.1 is known to work with Morse.

Once you have downloaded and untarred the two projects, you can simply build
them using cmake. 
For CERTI, you need to add the flag ``-DCERTI_USE_NULL_PRIME_MESSAGE_PROTOCOL=ON``
for better performance::

    $ cmake -DCERTI_USE_NULL_PRIME_MESSAGE_PROTOCOL=ON .

For pyHLA, do not forget to precise the python version you
want to build it using::

    $ cmake -DPYTHON_EXECUTABLE=/path/to/python3.{3,4} .

Using robotpkg
++++++++++++++

Using robotpkg, you can simply::

    $ cd robotpkg/wip/py-hla && make PREFER_ALTERNATIVE.python=python3{3,4} update


Back to Morse
+++++++++++++

MORSE installs by default HLA support, so as long as pyHLA is accessible from
your `PYTHONPATH`, you should be able to run HLA based simulation.

