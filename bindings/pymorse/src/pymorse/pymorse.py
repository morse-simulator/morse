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

    import time
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
        time.sleep(0.1)

        # waits until we reach the target
        while simu.r2d2.motion.get_status() != "Arrived":
            time.sleep(0.5)

        print("Here we are!")


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

    import time
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
            time.sleep(10)

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

    import time
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
            time.sleep(0.5)

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
import time
import socket
import logging
import asyncore
import asynchat
import threading
# Double-ended queue, thread-safe append/pop.
from collections import deque

from .future import MorseExecutor

logger = logging.getLogger("pymorse")
logger.setLevel(logging.WARNING)
# logger.addHandler( logging.NullHandler() )

MSG_SEPARATOR=b"\n"
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


class Component():
    def __init__(self, morse, name, fqn, stream = None, port = None, services = []):
        self._morse = morse
        self.name = name
        self.fqn = fqn # fully qualified name

        if port:
            self.stream = StreamJSON(morse.host, port)

            if stream == 'IN':
                self.publish = self.stream.publish
            elif stream == 'OUT':
                self.get = self.stream.get
                self.subscribe = self.stream.subscribe
                self.unsubscribe = self.stream.unsubscribe

        for service in services:
            logger.debug("Adding service %s to component %s" % (service, self.name))
            self._add_service(service)

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

    def close(self):
        if self.stream:
            self.stream.close()

class Robot(dict, Component):
    __getattr__ = dict.__getitem__
    __setattr__ = dict.__setitem__
    __delattr__ = dict.__delitem__

    def __init__(self, morse, name, fqn, services = []):
        Component.__init__(self, morse, name, fqn, None, None, services)

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
    _asyncore_thread = None
    def __init__(self, host = "localhost", port = 4000):
        """ Creates an instance of the MORSE simulator proxy.

        This is the main object you need to instanciate to communicate with the simulator.

        :param host: the simulator host (default: localhost)
        :param port: the port of the simulator socket interface (default: 4000)
        """
        self.host = host
        self.simulator_service = Stream(host, port)
        self.simulator_service_id = 0
        if not Morse._asyncore_thread:
            Morse._asyncore_thread = threading.Thread( target = asyncore.loop, kwargs = {'timeout': 0.01} )
            Morse._asyncore_thread.start()
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

    def _add_component(self, robot, fqn, details):
        stream = details.get('stream', None)
        port = None
        if stream:
            try:
                port = self.get_stream_port(fqn)
            except MorseServiceFailed:
                logger.warn('Component <%s> has a non-socket stream: datastream via pymorse not supported', fqn)
                stream = None

        services = details.get('services', [])

        name = fqn.split('.')[1:] # the first token is always the robot name. Remove it

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
            'args': ', '.join(json.dumps(arg) for arg in args),
        }
        self.simulator_service_id += 1
        return req

    def _rpc_process(self, req, timeout=None):
        raw = "{id} {component} {service} [{args}]".format(**req)
        logger.debug(raw)
        response_callback = ResponseCallback(req['id'])
        self.simulator_service.subscribe(response_callback.callback)
        try:
            self.simulator_service.publish(raw)
            with response_callback.condition:
                if self.is_up() and response_callback.condition.wait(timeout):
                    return rpc_get_result(response_callback.response)
        finally:
            self.simulator_service.unsubscribe(response_callback.callback)

        if not self.is_up():
            raise ConnectionError("simulation service is down")

        raise TimeoutError("timeout exceeded while waiting for response")

    def cancel(self, service_id):
        """ Send a cancelation request for an existing (running) service.

        If the service is not running or does not exist, the request is
        ignored.
        """
        self.simulator_service.publish("%i cancel"%int(service_id))

    def close(self, cancel_async_services = False):
        if cancel_async_services:
            logger.info('Cancelling all running asynchronous requests...')
            ResponseCallback.cancel_all()
            self.executor.cancel_all()
        else:
            logger.info('Waiting for all asynchronous requests to complete...')
        self.executor.shutdown(wait = True)
        self.simulator_service.close()
        # Close all other asyncore sockets (StreanJSON)
        asyncore.close_all()
        Morse._asyncore_thread.join(TIMEOUT)
        Morse._asyncore_thread = None # in case we want to re-create
        logger.info('Done. Bye bye!')

    def spin(self):
        Morse._asyncore_thread.join()


    #####################################################################
    ###### Predefined methods to interact with the simulator

    def quit(self):
        self.rpc("simulation", "quit")
        self.close()

    def reset(self):
       return self.rpc("simulation", "reset")

    def streams(self):
       return self.rpc("simulation", "list_streams")

    def get_stream_port(self, stream):
       return self.rpc("simulation", "get_stream_port", stream)

    def activate(self, cmpnt):
        return self.rpc("simulation", "activate", cmpnt)

    def deactivate(self, cmpnt):
        return self.rpc("simulation", "deactivate", cmpnt)

    #### with statement ####
    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if not exc_type:
            self.close()
        else:
            self.close(True)
            return False # re-raise exception

class Stream(asynchat.async_chat):
    """ Asynchrone I/O stream handler

    To start the handler, just run :meth asyncore.loop: in a new thread::

    threading.Thread( target = asyncore.loop, kwargs = {'timeout': .1} ).start()

    where timeout is used with select.select / select.poll.poll.
    """
    def __init__(self, host, port, maxlen=100):
        self.error = False
        asynchat.async_chat.__init__(self)
        self.set_terminator(MSG_SEPARATOR)
        self.create_socket(family=socket.AF_INET, type=socket.SOCK_STREAM)
        self.connect( (host, port) )
        self._in_buffer  = b""
        self._in_queue   = deque([], maxlen)
        self._callbacks  = []
        self._cv_new_msg = threading.Condition()

    def is_up(self):
        """ 
        self.connecting has been introduced lately in several branches
        of python (see issue #10340 of Python). In particular, it is not
        present in the python 3.2.3 interpreter delivered in Ubuntu 12.04.
        On this platform, just test of self.connected. There is still
        possibly a little race  but it mitigate the issue.
        """
        if hasattr(self, 'connecting'):
            return self.connecting or self.connected
        else:
            return self.connected

    def subscribe(self, callback):
        self._callbacks.append(callback)

    def unsubscribe(self, callback):
        self._callbacks.remove(callback)

    def handle_error(self):
        self.error = True
        self.handle_close()

    #### IN ####
    def collect_incoming_data(self, data):
        """Buffer the data"""
        self._in_buffer += data

    def found_terminator(self):
        self.handle_msg(self._in_buffer)
        self._in_buffer = b""

    def handle_msg(self, msg):
        """ append new raw :param msg: in the input queue

        and call subscribed callback methods if any
        """
        with self._cv_new_msg:
            self._in_queue.append(msg)
            self._cv_new_msg.notify_all()
        # handle callback(s)
        decoded_msg = None
        for callback in self._callbacks:
            if not decoded_msg:
                decoded_msg = self.decode( msg )
            callback( decoded_msg )

    def _msg_available(self):
        return bool(self._in_queue)

    def _get_last_msg(self):
        return self.decode( self._in_queue[-1] )

    # TODO implement last n msg decode and msg_queue with hash(msg) -> decoded msg
    def last(self, n=1):
        """ get the last message recieved

        :returns: decoded message or None if no message available
        """
        with self._cv_new_msg:
            if self._msg_available():
                return self._get_last_msg()
        logger.debug("last: no message in queue")
        return None

    def get(self, timeout=None):
        """ wait :param timeout: for a new messge

        When the timeout argument is present and not None, it should be a
        floating point number specifying a timeout for the operation in seconds
        (or fractions thereof).

        :returns: decoded message or None in case of timeout
        """
        with self._cv_new_msg:
            if self._cv_new_msg.wait(timeout):
                return self._get_last_msg()
        logger.debug("get: timed out")
        return None

    #### OUT ####
    def publish(self, msg):
        """ encode :param msg: and append the resulting bytes to the output queue """
        self.push(self.encode( msg ))
    #### patch code from asynchat, ``del deque[0]`` is not safe #####
    def initiate_send(self):
        while self.producer_fifo and self.connected:
            first = self.producer_fifo.popleft()
            # handle empty string/buffer or None entry
            if not first:
                if first is None:
                    self.handle_close()
                    return

            # handle classic producer behavior
            obs = self.ac_out_buffer_size
            try:
                data = first[:obs]
            except TypeError:
                data = first.more()
                if data:
                    self.producer_fifo.appendleft(data)
                continue

            if isinstance(data, str) and self.use_encoding:
                data = bytes(data, self.encoding)

            # send the data
            try:
                num_sent = self.send(data)
            except socket.error:
                self.handle_error()
                return

            if num_sent:
                if num_sent < len(data) or obs < len(first):
                    self.producer_fifo.appendleft(first[num_sent:])
            # we tried to send some actual data
            return


    #### CODEC ####
    def decode(self, msg_bytes):
        """ decode bytes to string """
        return msg_bytes.decode()

    def encode(self, msg_str):
        """ encode string to bytes """
        return msg_str.encode() + MSG_SEPARATOR

class StreamJSON(Stream):
    def __init__(self, host, port, maxlen=100):
        Stream.__init__(self, host, port, maxlen)

    def decode(self, msg_bytes):
        """ decode bytes to json object """
        return json.loads(Stream.decode(self, msg_bytes))

    def encode(self, msg_obj):
        """ encode object to json string and then bytes """
        return Stream.encode(self, json.dumps(msg_obj))
