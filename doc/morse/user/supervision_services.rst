Supervision services
====================

MORSE exposes several services to remotely manage and monitor the simulator.

These services are automatically exposed on a socket interface, on port 4000.

All these services belongs to a virtual component called ``simulation`` and
follows the normal syntax for socket requests.  Thus, a request to retrieve the
list of robots would look like that::

  > telnet localhost 4000
  Connected to localhost.
  > req1 simulation list_robots
  req1 OK ['Robot1', 'Robot2']

Or using ``pymorse``

.. code-block:: python

    import pymorse
    with pymorse.Morse() as sim:
        sim.rpc('simulation', 'get_camarafp_projection_matrix')
        sim.rpc('simulation', 'set_camarafp_far_clip', 1000)


Available services
------------------

- ``list_robots`` (no parameter): returns the list of the robots currently
  available in the simulation
- ``reset_objects`` (no parameter): reset the position of all objects in the
  simulation (in other word, restart the simulation)
- ``quit`` (no parameter): quit the game mode and so terminate the simulation
- ``activate`` (component_name): Activate the functionality of the specified component
- ``deactivate`` (component_name): Deactivate  the functionality of the specified component
- ``suspend_dynamics`` (no parameter): suspend the physics at the game engine
  level
- ``restore_dynamics`` (no parameter): enable again the physics at the game
  engine level
- ``details`` (no parameter): returns a structure containing the details about
  the simulation currently running, including the list of robots, the list of
  services and datastreams, ...
- ``set_log_level`` ``cmpnt`` (string) ``level`` (string): changes the
  level of logging for the component ``cmpnt`` to the level ``level``.
- ``get_scene_objects`` (no parameter): returns an hierarchical dictionary
  structure of all objects in the scene along with their positions and
  orientations (as quaternion).
- ``set_object_visibility`` ``cmpnt`` (string) ``visible`` (bool): make the
  object referenced by ``cmpnt`` visible or invisible (it still exists at
  physic level)
- ``set_object_dynamics`` ``cmpnt`` (string) ``state`` (bool): enable or
  disable the dynamics (physics) associated to component ``cmpnt``.
- ``set_object_position`` ``cmpnt`` (string) ``[x, y, z]`` (list) ``[roll
  pitch yaw]`` (optional list): move the object referenced by ``cmpnt`` to the
  specified ``position`` and optionally ``orientation``.
- ``set_camarafp_far_clip`` Set the CamaraFP (MORSE' environment camera) far
  clipping distance. (float)
- ``set_camarafp_position`` Set the CamaraFP world position. [x, y, z]
- ``set_camarafp_transform`` Set the CamaraFP world space 4x4 transform matrix.
- ``set_camarafp_projection_matrix`` Set the CamaraFP 4x4 projection matrix.
- ``get_camarafp_projection_matrix`` Get the CamaraFP 4x4 projection matrix.

.. note::
  Simulation services are stored in :py:mod:`morse.core.supervision_services`.
