#!/usr/bin/python
"""A Python interface to control the `MORSE <http://morse.openrobots.org>`_ robotics simulator

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

The data format is always formatted as a JSON dictionary (which means that, currently, binary data like images are not supported). The documentation page of each component specify the exact content of the dictionary.

Services
--------

Some components export RPC services. Please refer to the components' documentation for details.

These services can be accessed from `pymorse`, and mostly look like regular methods:

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

Non-blocking call are useful for long lasting services, like in the example below:

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
import os
import time
import logging
import socket
import select
import threading

from queue import Queue

from .future import MorseExecutor

import json

DEBUG_LEVEL=logging.DEBUG

class NullHandler(logging.Handler):
    """Defines a NullHandler for logging, in case pymorse is used in an application
    that doesn't use logging.
    """
    def emit(self, record):
        pass

pymorselogger = logging.getLogger("pymorse")
pymorselogger.setLevel(logging.DEBUG)

h = NullHandler()
pymorselogger.addHandler(h)


from .stream import Stream

class MorseServerError(Exception):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)

class MorseServiceFailed(Exception):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)

class MorseServicePreempted(Exception):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)


SUCCESS='SUCCESS'
FAILURE='FAILED'
PREEMPTED='PREEMPTED'

class Robot(dict):
    __getattr__= dict.__getitem__
    __setattr__= dict.__setitem__
    __delattr__= dict.__delitem__

class Component():
    def __init__(self, morse, name, fqn, stream = None, port = None, services = []):
        self._morse = morse
        self.name = name
        self.fqn = fqn # fully qualified name

        if stream == 'IN':
            self.stream = Stream(self._morse.com, port)
            self.publish = self.stream.publish
        elif stream == 'OUT':
            self.stream = Stream(self._morse.com, port)
            self.get = self.stream.get
            self.last = self.stream.last
            self.subscribe = self.stream.subscribe
            self.unsubscribe = self.stream.unsubscribe
        else:
            self.stream = None

        for s in services:
            pymorselogger.debug("Adding service %s to component %s" % (s, self.name))
            self._add_service(s)

    def _add_service(self, m):
        def innermethod(*args):
            pymorselogger.debug("Sending asynchronous request %s with args %s." % (m, args))
            req = self._morse._make_request(self.fqn, m, *args)
            future = self._morse.executor.submit(self._morse._execute_rpc, req)
            #TODO: find a way to throw an execption in the main thread
            # if the RPC request fails at invokation for stupid reasons
            # like wrong # of params
            return future

        innermethod.__doc__ = "This method is a proxy for the MORSE %s service." % m
        innermethod.__name__ = str(m)
        setattr(self,innermethod.__name__,innermethod)

    def close(self):
        if self.stream:
            self.stream.close()


class ComChannel(threading.Thread):

    id = 0

    def __init__(self, host):
        threading.Thread.__init__(self)

        self.host = host

        self._socks_lock = threading.Lock()
        self._socks = {}
        self._running = True

        # Creates a pair of pipe to wake up the
        # select when we need to write.
        self._read_fd, self._write_fd = os.pipe()

        pymorselogger.debug("Starting the communication thread now.")
        self.start()

    def run(self):
        """ This method reads and writes to/from the simulator.
        When a new request is pushed in _morse_requests_queue, it is send to the 
        server, when the server answer smthg, we check if it's a normal answer
        or an event, and dispatch accordingly the answer's content.
        """

        while self._running:

            try:
                # Only select'ing on inputs
                with self._socks_lock:
                    inputs = list(self._socks.keys())
                inputs.append(self._read_fd) # add the pipe to be able to wake up the select if needed
                inputready,outputready,exceptready = select.select(inputs, [], [])

            except select.error as e:
                break
            except socket.error as e:
                break

            with self._socks_lock:
                for sock, value in self._socks.items():
                    in_queue = value["in"]
                    out_queue = value["out"]
                    buf = value["buf"]
                    cb = value["cb"]
                    _buf = value["_buf"]

                    # Something to send to the server?
                    if not out_queue.empty():
                        r = out_queue.get()
                        if buf is not None: # stream connection!
                            raw = json.dumps(r) + "\n"
                        else: # RPC connection!
                            if 'special' in r:
                                raw = "{id} {special}\n".format(**r)
                            else:
                                if r['args']:
                                    r['args'] = ', '.join(json.dumps(a) for a in r['args'])
                                else:
                                    r['args'] = ''
                                raw = "{id} {component} {service} [{args}]\n".format(**r)

                        sock.send(raw.encode())

                    # Something to read from the server?
                    for i in inputready:
                        if i == sock:

                            server_dead = False
                            raw = _buf
                            while not '\n' in raw:
                                # Read how many bytes are available in the socket buffer
                                raw = sock.recv(4096).decode()

                                if not raw: # socket closed?
                                    server_dead = True
                                    break

                                _buf += raw


                            if not server_dead:
                                last_linefeed = _buf.rfind('\n')
                                rqsts = _buf[:last_linefeed].split('\n') # we keep only the complete requests (we may have more than one!)
                                self._socks[sock]["_buf"] = _buf[last_linefeed + 1:] # we keep the remaing part for next time

                                for data in rqsts:

                                    if buf is not None: # it's a stream connection!
                                        res = json.loads(data)
                                        in_queue.put(res)
                                        buf.appendleft(res)
                                        if cb:
                                            cb(res)
                                    else: # it's a RPC connection
                                        res = self.parse_response(data)
                                        in_queue.put(res)
                                        if cb:
                                            cb(res)

        with self._socks_lock:
            pymorselogger.debug('Closing the connections to MORSE...')
            for sock, value in self._socks.items():
                sock.close()

    def parse_response(self, raw):

        result = None
        try:
            id, status, result = raw.split(' ', 2)

            try:
                result = json.loads(result)
            except TypeError:
                pymorselogger.error("Could not deserialize MORSE answer! Got: <%s>" % result)
        except ValueError:
            # No return value
            if ' ' in raw:
                id, status = raw.split(' ')
            else:
                pymorselogger.error("Could not receive a valid response from MORSE: <%s>" % raw)
                id = '???'
                status = FAILURE

        morse_answer = {"status": status,
                        "id": id,
                        "result":result}

        pymorselogger.debug("Got answer: " + morse_answer['status'] + \
                ", " + str(morse_answer['result']))

        return morse_answer

    def process(self):
        """ make sure the write requests are processed by
        explicitely waking up select.
        """
        os.write(self._write_fd, b'1')

    def connect(self, port, read_queue, write_queue, buf = None, cb = None):
        """
        :param port: the socket port to connect to
        :param read_queue: the content read from the socket is pushed to this
          queue. Must be a queue.queue.
        :param write_queue: content to be sent to the simulator should be
           pushed to this queue. Must be a queue.queue.
        :param buf: (optional, default: None) if provided, content
          stored in the read_queue is copied as well in the buffer. Must support
          'appendleft()'. Attention: if 'buf' is provided, the connection is
          considered as a stream connection. Else as a RPC connection (using
          MORSE RPC protocol to parse the response).
        :param cb: (optional, default: None) a callback, invoked when content
          is read on the socket.

        """

        try:
            #create an INET, STREAMing socket
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

            #now connect to the morse server
            sock.connect((self.host, port))
        except socket.error:
            sock.close()
            raise MorseServerError('Unable to connect to MORSE on port %d. Check it is running, '
                                   'and the port is used by the simulation.' % port)

        with self._socks_lock:
            self._socks[sock] = {"in": read_queue,
                                "out": write_queue,
                                "cb": cb,
                                "buf": buf,
                                "_buf":""} # internal socket buffer
        # wake up select
        os.write(self._write_fd, b'1')

    def close(self):

        self._running = False
        # wake up select
        os.write(self._write_fd, b'1')
        self.join()
        pymorselogger.debug('Communications with MORSE are closed.')


class Morse():

    id = 0

    def __init__(self, host = "localhost", service_port = 4000):
        """ Creates an instance of the MORSE simulator proxy.
        
        This is the main object you need to instanciate to communicate with the simulator.
        
        :param host: the simulator host (default: localhost)
        :param port: the port of the simulator socket interface (default: 4000)
        """

        self._services_in_queue = Queue() # where we put the service result to read from MORSE
        self._services_out_queue = Queue() # where we put the request to send to MORSE

        self.com = ComChannel(host)

        # First, connect to the service port
        self.com.connect(service_port,
                         self._services_in_queue,
                         self._services_out_queue)

        self.executor = MorseExecutor(max_workers = 10, morse = self)

        self.initapi()

    def _normalize_name(self, name):
        """
        Normalize Blender names to get valid Python identifiers
        """
        new = name
        for c in ".-~":
            new = new.replace(c, "_")
        return new

    def initapi(self):
        """ This method asks MORSE for the scene structure, and
        dynamically creates corresponding objects in 'self'.
        """
        simu = self.rpc("simulation", "details")

        self.robots = []
        for r in simu["robots"]:
            name = self._normalize_name(r["name"])
            self.robots.append(name)
            setattr(self, name, Robot())
            robot = getattr(self, name)

            for c in sorted(r["components"].keys()): # important to sort the list of components to ensure parents are created before children
                self._add_component(robot, c, r["components"][c])

    def _add_component(self, robot, fqn, details):
        stream = details.get('stream', None)
        port = None
        if stream:
            try:
                port = self.get_stream_port(fqn)
            except MorseServiceFailed:
                pymorselogger.debug('Component <%s> has a non-socket stream: datastream via pymorse not supported', fqn)
                stream = None

        services = details.get('services', [])

        name = fqn.split('.')[1:] # the first token is always the robot name. Remove it

        cmpt = Component(self,
                        name[-1],
                        fqn,
                        stream,
                        port,
                        services)

        if len(name) == 1: # this component belongs to the robot directly.
            robot[name[0]] = cmpt
        else:
            subcmpt = robot[name[0]]
            for sub in name[1:-1]:
                subcmpt = getattr(subcmpt, sub)

            if hasattr(subcmpt, name[-1]): # pathologic cmpt name!
                raise RuntimeError("Sub-component name <%s> conflicts with <%s.%s> member. To use pymorse with this scenario, please change the name of the sub-component." % (name[-1], subcmpt.name, name[-1]))
            setattr(subcmpt, name[-1], cmpt)


    def cancel(self, id):
        """ Send a cancelation request for an existing (running) service.

        If the service is not running or does not exist, the request is
        ignored.
        """
        req = {'id': id,
               'special': 'cancel'}

        self._services_out_queue.put(req)
        self.com.process()


    def rpc(self, component, service, *args):
        """ Calls a service from the simulator.
        
        The call will block until a response from the simulator is received.
        
        :param component: the component that expose the service (like a robot name)
        :param service: the name of the service
        :param args...: (variadic) each service parameter, as a separate argument
        """
        return self._execute_rpc(self._make_request(component, service, *args))
        
    def _make_request(self, component, service, *args):
        req = {'id': str(self.id),
               'component': component,
               'service': service,
               'args': args}
        self.id += 1
        return req

    def _execute_rpc(self, req):

        self._services_out_queue.put(req)
        self.com.process()

        while True:
            res = self._services_in_queue.get() #block until we get an answer
            if res['id'] != req['id']:
                self._services_in_queue.put(res)
                time.sleep(0) # try to switch to another
            else:
                break

        if res['status'] == SUCCESS:
            if not res['result']:
                return None
            return res['result']
            
        elif res['status'] == FAILURE:
            msg = res['result']

            if msg and "wrong # of parameters" in msg:
                raise TypeError(msg)
            
            raise MorseServiceFailed(res['result'])

        elif res['status'] == PREEMPTED:

            msg = res['result']

            raise MorseServicePreempted(res['result'])
       
        else:
            raise MorseServerError("Got an unexpected message status from MORSE: " + \
            res['status'])

    def close(self, cancel_async_services = False):

        if cancel_async_services:
            pymorselogger.info('Cancelling all running asynchronous requests...')
            self.executor.cancel_all()
        else:
            pymorselogger.info('Waiting for all asynchronous requests to complete...')
        self.executor.shutdown(wait = True)
        self.com.close()
        pymorselogger.info('Done. Bye bye!')

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if not exc_type:
            self.close()
        else:
            self.close(True)
            return False # re-raise exception

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

