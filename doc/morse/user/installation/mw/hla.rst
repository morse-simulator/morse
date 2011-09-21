HLA
~~~

The High Level Architecture (HLA) is a standard framework that supports
simulations composed of different simulation components. Some introductory
courses about HLA are available `here <http://www.ecst.csuchico.edu/~hla/>`_.

The HLA implementation on which the multi-node version of MORSE is build is
the `CERTI <https://savannah.nongnu.org/projects/certi>`_. To install the CERTI,
follow the `Building CERTI <http://www.nongnu.org/certi/certi_doc/Install/html/build.html>`_
documentation.

The CVS version tagged "CERTI-MORSE-0_4" is the version tested at the moment of
the MORSE 0.4 release. If you are facing some mistakes with the head cvs version,
try to checkout the CERTI-MORSE-0_4 version::

$ cvs -z3 -d:pserver:anonymous@cvs.savannah.nongnu.org:/sources/certi checkout -r CERTI-MORSE-0_4 certi

You also have to create the corresponding Python binding in order to have
MORSE able to use the CERTI. The PyHLA binding can be installed following these
`instructions <http://www.nongnu.org/certi/PyHLA/manual/node6.html>`_.
Depending on your system configuration, you may have to configure PyHLA to use
the Python 3.2 executable and libraries.

Then you will have to update your PYTHONPATH so that MORSE will find the PyHLA
components.
