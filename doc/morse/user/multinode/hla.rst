HLA Multi-node Simulation with MORSE
====================================

General principle
-----------------

Multi-node infrastructure
+++++++++++++++++++++++++

The HLA multi-node infrastructure is made of a set of nodes, that are communicating
all together to perform a simulation. In this infrastructure, each node is a
MORSE node, i.e. a Blender application that simulates a MORSE scene as classicaly done
in a mono-node context.

Each node then manages a sub-set of the robots present in the simulated world.
"Managing a robot" means that the node will export the data of the robot's sensors,
and apply the commands of the robot's controller, from classical MORSE middlewares.

Each node is considering the other robots, that the node is not managing, as *External*:
an external robot is not updated by the node itself (robot's components are disbled),
but the robot's pose is updated from the HLA infrastructure in order to reflect
the robot's pose as it is simulated by its manager node.

The HLA infrastructure then ensures the consistency of all the simulated nodes:
at each time, all the robots' poses are equivalent in every node. Moreover,
it decreases the processing load of each node, that only has to process the
sensor/command of the managed robots.

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


Configuring its HLA environment
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

Setting up an HLA simulation using the distribution script
---------------------------------------------------------

MORSE provides a distribution script that helps setting up a multi-node simulation.
The principle of this script is that you just have to define your MORSE scenario
as a mono-node scenario, i.e. including all robots, sensors, controlers in the
same scene.

Then, from this scenario and a configuration file, MORSE will generate one script
file per node that sets up the MORSE/HLA component.

Creating the global scenario
++++++++++++++++++++++++++++

For the moment, the distribution script only works with scenarios defined as
Python script using the MORSE Builder API.

The configuration file
++++++++++++++++++++++

The configuration file used by the distribution script is a text file containing:

* a ``[global]`` section, where you can define some HLA and MORSE parameters:

    - **host**: the ``CERTI_HOST`` value (see `Configuring its HLA environment`_)
    - **proxy**: the ``CERTI_HTTP_PROXY`` value (see `Configuring its HLA environment`_)
    - **time**: whether to activate time synchronization or not; the assigned 
        value can be one of *on*, *off*, *yes*, *no*;
    - **debug**: whether MORSE will display debug values or not;
    - **profile**: whether MORSE will display profiling values or not.
    
* a section per node, called ``[node_name]``, containing the list of managed robots.
    Each line will have the syntax **Robot_name:** *value*, where value can be:
    
    - *on* or *yes*: the robot with name **Robot_name** will be managed by the node;
    - *off* or *no*: the robot with name **Robot_name** will be external to the node.
    
    If no value is indicated, *yes* is considered by default.
    If a robot name is not listed in a node section, *no* is considered by default.
    
Here is an example of a configuration file::

    [global]
    host: 134.212.24.32
    time: off
    profile: on
    
    [GroundNode]
    Dala: on
    Mana
    
    [AerialNode]
    Ressac: yes
    Mana: no
    
In this configuration file, the CERTI host is set to IP 134.212.24.32. Time
syncrhonization is off, and MORSE will display profiling values.

The simulation is made of two nodes, *GroundNode* and *AerialNode*.
The *GroundNode* node will manage both robots *Mana* and *Dala*.
The *AerialNode* will only manage *Ressac*.
    
Generating the node scripts
+++++++++++++++++++++++++++

The MORSE distribution script is located in the ``tools`` directory of the MORSE
sources. Then just launch::

$ ./create_nodes <config_file> <scenario>

The script will create one scenario per node. For example, if you have provided
the configuration file described above, the script will create a 
``scenario_GroundNode.py`` file, to be executed by the *GroundNode* node, and
a ``scenario_AerialNode.py`` file, for the *AerialNode* node.

These scripts imports the original scenario file, and then add HLA specific
instructions. They disable the node external robots, and they add the HLA
component to the scene and configure it as specified in the configuration file.

The MORSE HLA component
-----------------------

HLA is managed in MORSE using the *HLA_Empty* component.
One present in a scene, this component is responsible of exporting managed
robot poses to the HLA simulation, and of updating external robot poses from the
simulation.

If you want to manually add this component, it is located in the 
``$MORSE_ROOT/share/data/morse/components/middleware/hla_empty.blend`` file.

The HLA component has a set of properties that can be modified as any other 
MORSE component. These properties have default values, and are configured 
according to a configuration file if you used the automatic distribution script.

Executing an HLA multi-node simulation
--------------------------------------

Once you have generated your node scenarios, you are ready to launch the 
multi-node simulation.

1. Launch the RTIG on the RTIG host machine; the CERTI command is ``rtig``.
2. On each node, launch the distributed scenario, using::

$ morse exec <scenario_node.py>

3. Then, on each node, launch the game engine by pressing *P*.

The nodes are now synchronized: move one robot on the node where it is
managed, its pose will be reflected on the other nodes.

To start with multi-node simulation, you can try the HLA Tutorial (coming soon).
