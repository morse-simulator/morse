#!/usr/bin/env python3
"""A Python interface to control `MORSE <http://morse.openrobots.org>`_,
*the robotics simulator*.

The ``pymorse`` library exposes MORSE services and data stream with a
friendly Python API.

It uses underneath the MORSE socket API.

Usage
=====

Creating a connection to the simulator
--------------------------------------

- Import the ``pymorse`` module
- Create a context with the ``with`` statement:

.. code-block:: python

    import pymorse

    with pymorse.Morse() as simu:
        # ...
        pass


The context manager will take care of properly closing the connection to the
simulator.  You can also directly create an instance of the
:py:class:`pymorse.Morse` class, passing the host and/or port of the
simulator (defaults to localhost:4000). In this case, you must call
:py:meth:`pymorse.Morse.close` before leaving.

- Once created, the context generates proxies for every robots in the scene,
  and every sensors and actuators for each robot.

First example
-------------

Let consider MORSE has been started with the following simulation script:

.. code-block:: python

    from morse.builder import *

    # Add a robot with a position sensor and a motion controller
    r2d2 = ATRV()

    pose = Pose()
    pose.add_interface('socket')
    r2d2.append(pose)

    motion = Waypoint()
    motion.add_interface('socket')
    r2d2.append(motion)


    # Environment
    env = Environment('land-1/trees')

The following Python program sends a destination to the robot, and prints in
background its pose:

.. code-block:: python

    import pymorse

    def print_pos(pose):
        print("I'm currently at %s" % pose)

    with pymorse.Morse() as simu:

        # subscribes to updates from the Pose sensor by passing a callback
        simu.r2d2.pose.subscribe(print_pos)

        # sends a destination
        simu.r2d2.motion.publish({'x' : 10.0, 'y': 5.0, 'z': 0.0,
                                  'tolerance' : 0.5,
                                  'speed' : 1.0})

        # Leave a couple of millisec to the simulator to start the action
        simu.sleep(0.1)

        # waits until we reach the target
        while simu.r2d2.motion.get_status() != "Arrived":
            simu.sleep(0.5)

        print("Here we are!")

.. note::

    Note that here we use ``pymorse.Morse.sleep()`` instead of standard
    ``time.sleep``, the former allowing to consider the simulation time
    while the second use 'system' time.


Data stream manipulation
------------------------

Every component (sensor or actuator) exposes a datastream interface,
either read-only (for sensors) or write-only (for actuators)

They are accessible by their names, as defined in the simulation
script (cf example above).

Streams are Python :py:class:`pymorse.Stream` objects. It offers several
methods to read data:

- :py:meth:`pymorse.Stream.get`: blocks until new data are
  available and returns them.
- :py:meth:`pymorse.Stream.last`: returns the last/the n last (if
  an integer argument is passed) records received, or none/less, if
  none/less have been received.
- :py:meth:`pymorse.Stream.subscribe` (and
  :py:meth:`pymorse.Stream.unsubscribe`): this method is called
  with a callback that is triggered everytime incoming data is received.

These methods are demonstrated in the example below:

.. code-block:: python

    import pymorse

    def printer(data):
        print("Incoming data! " + str(data))

    with pymorse.Morse("localhost", 4000) as simu:

        try:
            # Get the 'Pose' sensor datastream
            pose = simu.r2d2.pose

            # Blocks until something is available
            print(pose.get())

            # Asynchronous read: the following line do not block.
            pose.subscribe(printer)

            # Read for 10 sec
            simu.sleep(10)

        except pymorse.MorseServerError as mse:
            print('Oups! An error occured!')
            print(mse)

Writing on actuator's datastreams is achieved with the
:py:meth:`pymorse.Stream.publish` method, as illustrated in the first example
above.

The data format is always formatted as a JSON dictionary (which means that,
currently, binary data like images are not supported). The documentation page
of each component specify the exact content of the dictionary.

Services
--------

Some components export RPC services. Please refer to the components'
documentation for details.

These services can be accessed from `pymorse`, and mostly look like regular
methods:

.. code-block:: python

    import pymorse

    with pymorse.Morse() as simu:

        # Invokes the get_status service from the 'Waypoints' actuator
        print(simu.r2d2.motion.get_status())

However, these call are **asynchronous**: a call to
`simu.r2d2.motion.get_status()` does not block, and returns instead a
`future`. See the `concurrent.futures API
<http://docs.python.org/dev/library/concurrent.futures.html>`_ to learn more
about `futures`.

Non-blocking call are useful for long lasting services, like in the example
below:

.. code-block:: python

    import pymorse

    def done(evt):
        print("We have reached our destination!")

    with pymorse.Morse() as simu:

        # Start the motion. It may take several seconds before finishing
        # The line below is however non-blocking
        goto_action = simu.r2d2.motion.goto(2.5, 0, 0)

        # Register a callback to know when the action is done
        goto_action.add_done_callback(done)

        print("Am I currently moving? %s" % goto_action.running())

        while goto_action.running():
            simu.sleep(0.5)

Use the `cancel` method on the `future` returned by the RPC call to
abort the service.

To actually wait for a result, call the `result` method on the future:

.. code-block:: python

    import pymorse

    with pymorse.Morse() as simu:

        status = simu.r2d2.motion.get_status().result()

        if status == "Arrived":
            print("Here we are")

However, for certain common cases (printing or comparing the return value), the
`result()` method is automatically called. Thus, the previous code sample can
be rewritten:

.. code-block:: python

    import pymorse

    with pymorse.Morse() as simu:

        if simu.r2d2.motion.get_status() == "Arrived":
            print("Here we are")


Simulator control
-----------------

Several services are available to control the overall behaviour of the
simulator.

The whole list of such services is here: `Supervision services
<http://www.openrobots.org/morse/doc/latest/user/supervision_services.html>`_.

For instance, you can stop the simulator (MORSE will quit) with
:py:meth:`pymorse.quit`, and reset it with :py:meth:`pymorse.Morse.reset`.

These methods are demonstrated in the example below:

.. code-block:: python

    import pymorse

    with pymorse.Morse("localhost", 4000) as simu:

        try:
            print(simu.robots)
            simu.quit()

        except pymorse.MorseServerError as mse:
            print('Oups! An error occured!')
            print(mse)

Logging
=======

This library use the standard Python logging mechanism.  You can
retrieve pymorse log messages through the "pymorse" logger.

The complete example below shows how to retrieve the logger and how to
configure it to print debug messages on the console.

.. code-block:: python

    import logging
    import pymorse

    logger = logging.getLogger("pymorse")

    console = logging.StreamHandler()
    console.setLevel(logging.DEBUG)
    formatter = logging.Formatter('%(asctime)-15s %(name)s: %(levelname)s - %(message)s')
    console.setFormatter(formatter)

    logger.addHandler(console)

    with pymorse.Morse("localhost", 4000) as simu:

        try:
            print(simu.robots)

            simu.quit()

        except pymorse.MorseServerError as mse:
            print('Oups! An error occured!')
            print(mse)
"""
import json
import logging
import asyncore
import threading
import re

from .future import MorseExecutor
from .stream import Stream, StreamJSON, PollThread

logger = logging.getLogger("pymorse")
logger.setLevel(logging.WARNING)
# logger.addHandler( logging.NullHandler() )

TIMEOUT=8
BUFFER_SIZE=8192
SUCCESS='SUCCESS'
FAILURE='FAILED'
PREEMPTED='PREEMPTED'

class MorseServiceError(Exception):
    """ Morse Service Exception thrown when unknown error """

class MorseServiceFailed(Exception):
    """ Morse Service Exception thrown when failed error """

class MorseServicePreempted(Exception):
    """ Morse Service Exception thrown when preempted error """


class Component(object):
    def __init__(self, morse, name, fqn, stream = None, port = None, services = []):
        self._morse = morse
        self.name = name
        self.fqn = fqn # fully qualified name
        
        self.stream = None
        self._init = False
        self._port = port
        if not stream:
            self._stream_dir = set()
        else:
            self._stream_dir = set([s[1] for s in stream])

        for service in services:
            logger.debug("Adding service %s to component %s" % (service, self.name))
            self._add_service(service)

    def lazy_init(self):
        if self._init:
            return

        if self._port:
            self.stream = StreamJSON(self._morse.host, self._port)

            if 'IN' in self._stream_dir:
                self.publish = self.stream.publish
            if 'OUT' in self._stream_dir:
                self.get = self.stream.get
                self.last = self.stream.last
                self.subscribe = self.stream.subscribe
                self.unsubscribe = self.stream.unsubscribe

        self._init = True

    def _add_service(self, method):
        def innermethod(*args):
            logger.debug("Sending asynchronous request %s with args %s." % (method, args))
            req = self._morse._rpc_request(self.fqn, method, *args)
            future = self._morse.executor.submit(self._morse._rpc_process, req)
            #TODO: find a way to throw an execption in the main thread
            # if the RPC request fails at invokation for stupid reasons
            # like wrong # of params
            return future

        innermethod.__doc__ = "This method is a proxy for the MORSE %s service." % method
        innermethod.__name__ = str(method)
        setattr(self, innermethod.__name__, innermethod)

    def __getattribute__(self, name):
        comp = object.__getattribute__(self, name)
        if hasattr(comp, 'lazy_init'):
            comp.lazy_init()
        return comp

    def close(self):
        if self.stream:
            self.stream.close()

class Robot(dict, Component):
    __setattr__ = dict.__setitem__
    __delattr__ = dict.__delitem__

    def __init__(self, morse, name, fqn, services = []):
        Component.__init__(self, morse, name, fqn, None, None, services)

    def __getattr__(self, name):
        comp = dict.__getitem__(self, name)
        if hasattr(comp, 'lazy_init'):
            comp.lazy_init()
        return comp

def normalize_name(name):
    """Normalize Blender names to get valid Python identifiers"""
    normalized = name
    for illegal in ".-~":
        normalized = normalized.replace(illegal, "_")
    return normalized

def parse_response(raw):
    result = None
    try:
        msg_id, status, result = raw.split(' ', 2)
        try:
            result = json.loads(result)
        except TypeError:
            logger.error("Could not deserialize MORSE answer! Got: <%s>" % result)
    except ValueError:
        # No return value
        if ' ' in raw:
            msg_id, status = raw.split(' ')
        else:
            logger.error("Could not receive a valid response from MORSE: <%s>" % raw)
            msg_id = '???'
            status = FAILURE

    logger.debug("Got answer: %s, %s"%(status, result))
    return {
        "id": msg_id,
        "status": status,
        "result": result,
    }

def rpc_get_result(response):
    result = response['result']
    status = response['status']

    if status == SUCCESS:
        return result
    elif status == FAILURE:
        if result and "wrong # of parameters" in result:
            raise TypeError(result)
        raise MorseServiceFailed(result)
    elif status == PREEMPTED:
        raise MorseServicePreempted(result)
    else:
        raise MorseServiceError(result)

class ResponseCallback:
    _conditions = []
    def __init__(self, req_id):
        self.request_id = req_id
        self.response = None
        self.condition = threading.Condition()
        ResponseCallback._conditions.append(self.condition)

    def callback(self, res_bytes):
        response = parse_response(res_bytes)
        if response['id'] == self.request_id:
            self.response = response
            with self.condition:
                ResponseCallback._conditions.remove(self.condition)
                self.condition.notify_all()

    def cancel_all():
        for condition in ResponseCallback._conditions:
            with condition:
                condition.notify_all()
        del ResponseCallback._conditions[:] # clear list

class Morse(object):
    poll_thread = None
    def __init__(self, host = "localhost", port = 4000):
        """ Creates an instance of the MORSE simulator proxy.

        This is the main object you need to instanciate to communicate with the simulator.

        :param host: the simulator host (default: localhost)
        :param port: the port of the simulator socket interface (default: 4000)
        """
        self.host = host
        self.simulator_service = Stream(host, port)
        self.simulator_service_id = 0
        if not Morse.poll_thread:
            Morse.poll_thread = PollThread()
            Morse.poll_thread.start()
            logger.debug("Morse thread started")
        else:
            logger.debug("Morse thread was already started")
        self.executor = MorseExecutor(max_workers = 10, morse = self)
        self.initialize_api()

    def is_up(self):
        return self.simulator_service.is_up()

    def initialize_api(self):
        """ This method asks MORSE for the scene structure, and
        dynamically creates corresponding objects in 'self'.
        """
        details = self.rpc_t(15, 'simulation', 'details') # RPC with timeout
        if not details:
            raise ValueError("simulation details not available")
        logger.debug(details)
        self.robots = []
        for robot_detail in details["robots"]:
            name = normalize_name(robot_detail["name"])
            self.robots.append(name)
            robot = Robot(self, robot_detail['name'], robot_detail['name'],
                          services = robot_detail.get('services', []))
            setattr(self, name, robot)

            components = robot_detail["components"]
            # important to sort the list of components to ensure parents are
            # created before children
            for component in sorted(components.keys()):
                self._add_component(robot, component, components[component])

        # Handle robots created in loop. Basically, consider robot where
        # name match the pattern 'robot_XXX' and puts them in a list
        # called 'robots', allowing to iterate easily on them
        robot_names = self.robots[:]
        robot_names.sort()
        while robot_names:
            name = robot_names.pop(0)
            regexp_name = "^" + name + "_[0-9]{3}$"
            regexp = re.compile(regexp_name)
            loop_name = [name for name in robot_names if regexp.match(name)]
            if loop_name:
                list_robots = []
                list_robots.append(getattr(self, name))
                for _name in loop_name:
                    list_robots.append(getattr(self, _name))
                    robot_names.remove(_name)
                setattr(self, name + "s", list_robots)

    def _add_component(self, robot, fqn, details):
        stream = details.get('stream_interfaces', None)
        port = None
        if stream:
            try:
                port = self.get_stream_port(fqn)
            except MorseServiceFailed:
                logger.warn('Component <%s> has a non-socket stream: datastream via pymorse not supported', fqn)
                stream = None

        services = details.get('services', [])

        name = fqn.split('.')[1:] # the first token is always the robot name. Remove it

        if not name:
            logger.error("Component <%s> of robot <%s> has an invalid name! " \
                         "Please report this bug on morse-users@laas.fr" % (fqn, robot.name))
            return

        logger.debug("Component %s" % str((name[-1], fqn, stream, port, services)) )
        cmpt = Component(self, name[-1], fqn, stream, port, services)

        if len(name) == 1: # this component belongs to the robot directly.
            robot[name[0]] = cmpt
        else:
            subcmpt = robot[name[0]]
            for sub in name[1:-1]:
                subcmpt = getattr(subcmpt, sub)

            if hasattr(subcmpt, name[-1]): # pathologic cmpt name!
                raise RuntimeError("Sub-component name <%s> conflicts with"
                        "<%s.%s> member. To use pymorse with this scenario,"
                        "please change the name of the sub-component." %
                        (name[-1], subcmpt.name, name[-1]))
            setattr(subcmpt, name[-1], cmpt)

    def rpc_t(self, timeout, component, service, *args):
        req = self._rpc_request(component, service, *args)
        return self._rpc_process(req, timeout)

    def rpc(self, component, service, *args):
        """ Calls a service from the simulator.

        The call will block until a response from the simulator is received.

        :param component: the component that expose the service (like a robot name)
        :param service: the name of the service
        :param args...: (variadic) each service parameter, as a separate argument
        """
        req = self._rpc_request(component, service, *args)
        return self._rpc_process(req)

    def _rpc_request(self, component, service, *args):
        req = {
            'id': '%i'%self.simulator_service_id,
            'component': component,
            'service': service,
            'args': json.dumps(args),
        }
        self.simulator_service_id += 1
        return req

    def _rpc_process(self, req, timeout=None):
        raw = "{id} {component} {service} {args}".format(**req)
        logger.debug(raw)
        response_callback = ResponseCallback(req['id'])
        self.simulator_service.subscribe(response_callback.callback)
        try:
            with response_callback.condition:
                self.simulator_service.publish(raw)
                # if self.is_up() and response_callback.condition.wait(timeout):
                # XXX Python 3.2.2 is_up() returns False when connecting...
                response_callback.condition.wait(timeout)
                if response_callback.response:
                    return rpc_get_result(response_callback.response)
        finally:
            self.simulator_service.unsubscribe(response_callback.callback)

        if not self.is_up():
            raise RuntimeError("simulation service is down")

        raise RuntimeError("timeout exceeded while waiting for response")

    def cancel(self, service_id):
        """ Send a cancelation request for an existing (running) service.

        If the service is not running or does not exist, the request is
        ignored.
        """
        self.simulator_service.publish("%i cancel"%int(service_id))

    def get_publisher_streams(self):
        for name in self.robots:
            for elt in getattr(self, name).values():
                if type(elt) is Component and 'publish' in dir(elt):
                    yield elt.stream

    def close(self, cancel_async_services = False, wait_publishers = True):
        if wait_publishers:
            import time
            for stream in self.get_publisher_streams():
                while len(stream.producer_fifo) > 0:
                    time.sleep(0.001)
        if cancel_async_services:
            logger.info('Cancelling all running asynchronous requests...')
            ResponseCallback.cancel_all()
            self.executor.cancel_all()
        else:
            logger.info('Waiting for all asynchronous requests to complete...')
        self.executor.shutdown(wait = True)
        # Close all other asyncore sockets (StreanJSON)
        if Morse.poll_thread:
            Morse.poll_thread.syncstop(TIMEOUT)
        asyncore.close_all()
        Morse.poll_thread = None # in case we want to re-create
        logger.info('Done. Bye bye!')

    def spin(self):
        Morse.poll_thread.join()


    #####################################################################
    ###### Predefined methods to interact with the simulator

    def quit(self):
        self.rpc("simulation", "quit")
        self.close()

    def reset(self):
       return self.rpc("simulation", "reset_objects")

    def streams(self):
       return self.rpc("simulation", "list_streams")

    def get_stream_port(self, stream):
       return self.rpc("simulation", "get_stream_port", stream)

    def activate(self, cmpnt):
        return self.rpc("simulation", "activate", cmpnt)

    def deactivate(self, cmpnt):
        return self.rpc("simulation", "deactivate", cmpnt)

    def sleep(self, time):
        """ Wait for time second.

        Time may be a float. Contrary to ``time.sleep``, this method
        consider the simulated time.
        """
        return self.rpc("time", "sleep", time)

    def time(self):
        """ Return the simulated time, in seconds, since Epoch

        The precision of the value depends on the underlying python
        precision of time, and the frequency of simulator
        """
        return self.rpc("time", "now")

    #### with statement ####
    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if not exc_type:
            self.close()
        else:
            self.close(True)
            return False # re-raise exception
