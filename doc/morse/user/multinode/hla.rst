Multi-node Simulation using HLA
===============================

Specific functionality for HLA
------------------------------

Time synchronization
++++++++++++++++++++

The HLA multi-node simulation offers two ways of synchronizing the simulation nodes.

**Best effort** modes is the simplest synchronization mode. Each node provides
its robots' states as fast as it can (once per Blender frame), and updates the
external robots' states as it receives them. The nodes are not really synchronized:
nothing guarantees that the node worlds are the same.
Moreover, the Blender engines are not constrained: each Blender node runs as fast
as it can, then leading to different simulation rates on each node.
However, if your network is fast enough, and if you do not rely on the Blender logical
time for your simulation, this multi-node simulation mode should be sufficient
for most cases.

**Time synchronization** ensures that all the nodes will have the same time at each
moment. Here, understand *logical time*, or *Blender time*: the number of frames executed
each second on each Blender node will be identical. This simulation mode guarantees
that all the nodes will have the same simulation state at each time step.


Configuring the HLA environment
-------------------------------

The CERTI implementation of HLA is based on a process, the RTIG (RTI - Run Time 
Infrastructure - Gateway), that manages the simulation and transfers messages between
nodes.

In order to execute an HLA simulation, you have to configure your system environment.
The `CERTI documentation <http://www.nongnu.org/certi/certi_doc/User/html/execute.html>`_
gives some information about the configurable variables. The environment variables 
that may be relevant to MORSE are described below:

* ``CERTI_HOST``: each computer hosting a MORSE node must have this variable pointing 
  to the host (IP address or host name) where the RTIG will be executed.
  
* ``CERTI_HTTP_PROXY``: if you have to use a proxy in order to join the RTIG, you 
  have to indicate it using the ``CERTI_HTTP_PROXY`` variable; if you proxy is already 
  defined in the ``http_proxy`` variable, please, reset it using::
  
  $ export CERTI_HTTP_PROXY=$http_proxy

  The ``http_proxy`` variable is not used in MORSE/HLA as, most of the time, it
  causes some simulation mistakes.
  
  .. note::
        The ``CERTI_HOST`` and ``CERTI_HTTP_PROXY`` variables can be defined either
        directly on MORSE nodes (see `The MORSE HLA component`_) or using the 
        distribution script (see `Setting up an HLA simulation using the distribution 
        script`_)

* ``CERTI_FOM_PATH``: this variable is only relevant for the RTIG. It indicates
  where to find the FOM file, that lists all the messages that can be exchanged
  on the HLA simulation. This FOM file is installed with MORSE, so if you have
  installed MORSE on the computer where you launch the RTIG, you have to
  define this variable as::
  
  $ export CERTI_FOM_PATH=$MORSE_ROOT/share/federations:$CERTI_FOM_PATH
  
  Otherwise, you will have to copy the ``morse.fed`` file from the 
  ``src/morse/middleware/hla`` source directory to your computer and define the
  ``CERTI_FOM_PATH`` variable accordingly. Default directories where the RTIG
  is looking for FOM files are the working directory (from where you launched the RTIG)
  and the ``$CERTI_HOME/share/federations`` path.


Executing an HLA multi-node simulation
--------------------------------------

Once you have generated your node scenarios, you are ready to launch the 
multi-node simulation.

1. Launch the RTIG on the RTIG host machine; the CERTI command is ``rtig``.

2. On each node, launch the distributed scenario, using::

    $ morse exec <scenario_node.py>

3. Then, on each node, launch the game engine by pressing :kbd:`p`.

The nodes are now synchronized: move one robot on the node where it is
managed, its pose will be reflected on the other nodes.

To start with multi-node simulation, you can try the HLA Tutorial (coming soon).
