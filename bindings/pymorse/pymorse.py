#!/usr/bin/python
"""A Python interface to control the `MORSE <http://morse.openrobots.org>`_ robotics simulator

The ``pymorse`` library exposes MORSE services and data stream with a
friendly Python API.

It uses underneath the MORSE socket API.

Usage
=====

.. note ::
  The current version of pymorse only support a largely incomplete set
  of features.  You may need to directly access the MORSE socket
  interface to perform certain actions like writing on a data stream.


General usage
-------------

- Import the ``pymorse`` module
- Create a context with the ``with`` statement:

.. code-block:: python

    import pymorse

    with pymorse.Morse() as morse:
        # ...
        pass


The context manager will take care of properly closing the connection to the
simulator.  You can also directly create an instance of the
:py:class:`pymorse.Morse` class, passing the host and/or port of the
simulator (defaults to localhost:4000). In this case, you must call
:py:meth:`pymorse.Morse.close` before leaving.

- Play with it!

MORSE control
-------------

Once the simulator is started, you can query the list of all robots
present in the simulation with :py:meth:`pymorse.Morse.robots`.

You can stop the simulator (MORSE will quit) with
:py:meth:`pymorse.quit`, and reset it with :py:meth:`pymorse.Morse.reset`.

These methods are demonstrated in the example below:

.. code-block:: python

    import pymorse

    with pymorse.Morse("localhost", 4000) as morse:

        try:
            print(str(morse.robots()))
            morse.quit()

        except MorseServerError as ose:
            print('Oups! An error occured!')
            print(ose)

Accessing services
------------------

No high level interface is currently supported by this version of ``pymorse``.

Services can be invoked in a blocking way with 
:py:meth:`pymorse.Morse.call_server`.

Reading a data stream
---------------------

The :py:meth:`pymorse.Morse.streams` method returns the list of the
names of available data streams from the simulator.

A :py:class:`pymorse.MorseStream` object can be then created with
:py:meth:`pymorse.Morse.stream` by passing the name of the stream you
want to subscribe to.

:py:class:`pymorse.MorseStream` proposes several methods to read the
stream:

- :py:meth:`pymorse.MorseStream.get`: blocks until new data are
  available and returns them.
- :py:meth:`pymorse.MorseStream.last`: returns the last/the n last (if
  an integer argument is passed) records received, or none/less, if
  none/less have been received.
- :py:meth:`pymorse.MorseStream.subscribe` (and
  :py:meth:`pymorse.MorseStream.unsubscribe`): this method is called
  with a callback that is triggered everytime incoming data is received.

These methods are demonstrated in the example below:

.. code-block:: python

    import time
    import pymorse

    def printer(data):
        print("Incoming data! " + str(data))

    with pymorse.Morse("localhost", 4000) as morse:

        try:
            streams = morse.streams()

            # Take arbitrary the first one
            mystream = morse.stream(streams[0])

            # Blocks until something is available
            print(str(mystream.get()))

            # Asynchronous read: the following line do not block.
            mystream.subscribe(printer)

            # Read for 10 sec
            time.sleep(10)

        except MorseServerError as ose:
            print('Oups! An error occured!')
            print(ose)


Writing to a data stream
------------------------

This is not supported by this version of ``pymorse``.

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

    with pymorse.Morse("localhost", 4000) as morse:

        try:
            print(str(morse.robots()))

            morse.quit()

        except MorseServerError as ose:
            print('Oups! An error occured!')
            print(ose)
"""
import time
import logging
import socket
import select
import threading

try:
    from queue import Queue
except ImportError:
    # Python 2.x
    from Queue import Queue

# Double-ended queue, thread-safe append/pop.
from collections import deque

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

SUCCESS='SUCCESS'
FAILURE='FAILED'

class MorseServerError(Exception):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)

class MorseStream(threading.Thread):

    def __init__(self, host, port, readonly = True, maxlength = 100):
        threading.Thread.__init__(self)

        self.maxlength = maxlength
        self._queue = deque([], maxlength)
        self.readonly = readonly
        self._running = True

        self.host = host
        self.port = port

        # Lock required for proper slicing of the queue (when returning 
        # the n latest records).
        # Condition variable required for the blocking get() method.
        self.cv = threading.Condition()

        self.subscribers = []

        self.start()

    def __del__(self):
        self.close()
    
    def close(self):
        self._running = False
        self.join()

    def run(self):
        """ Reads a data stream on a socket, convert it to Python object, and store it
        in a ring buffer of length self.maxlength.
        """
        _client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        _client.connect((self.host, self.port))

        stream = None

        if self.readonly:
            stream = _client.makefile('r')
            pymorselogger.info("Connected to a MORSE stream (read only).")
        else:
            stream = _client.makefile('rw')
            pymorselogger.info("Connected to a MORSE stream (read and write).")

        while self._running:
            try:
                record = json.loads(stream.readline().rstrip('\n'))
            except ValueError:
                pymorselogger.error("Received invalid JSON content from MORSE."
                                    " The connection has been maybe lost. Closing"
                                    " the stream.")
                break

            self.cv.acquire()
            self._queue.appendleft(record)
            self.cv.notify()
            self.cv.release()

            # Attention: call-back subscribers are called from the
            # reading thread. Better to keep them short!
            for cb in self.subscribers:
                cb(record)

        stream.close()

        pymorselogger.info("Shutting down the socket...")
        _client.shutdown(socket.SHUT_RDWR)
        pymorselogger.info("Closing socket client...")
        _client.close()


    def last(self, n = None):
        """ Returns the latest or the n latest data records read
        from the data stream.

        If called without parameter, returns a single record, as a
        Python object. If no record has been received yet, returns None.
        
        If called with a parameter n, returns the n latest ones or less
        if less records have been received.
        """
        if not self._queue:
            return None

        if not n:
            self.cv.acquire()
            res = self._queue[0]
            self.cv.release()

        else:
            n = min(n, self.maxlength)
            self.cv.acquire()
            res = list(self._queue)[:n] #TODO: optimize that! currently, the whole queue is copied :-/
            self.cv.release()

        return res

    def get(self):
        """ Blocks until a new record is read from the data stream, and returns it.
        """
        self.cv.acquire()
        self.cv.wait()
        res = self._queue[0]
        self.cv.release()

        return res

    def subscribe(self, cb):
        self.subscribers.append(cb)

    def unsubscribe(self, cb):
        self.subscribers.remove(cb)

class Morse(threading.Thread):

    id = 0

    def __init__(self, host = "localhost", port = 4000):
        """ Creates an instance of the MORSE simulator proxy.
        
        This is the main object you need to instanciate to communicate with the simulator.
        
        :param host: the simulator host (default: localhost)
        :param port: the port of the simulator socket interface (default: 4000)
        """
        threading.Thread.__init__(self)

        self._morse_requests_queue = Queue()
        self._morse_responses_queue = Queue()

        self.open_streams = [] #Keep an eye on open streams in case we need to close them
        self._running = True

        self._morse_server = None

        self.host = host
        try:
            #create an INET, STREAMing socket
            self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

            #now connect to the morse server
            self.s.connect((host, port))
            self._morse_server = self.s.makefile('rw')
        except socket.error:
            self.s.close()
            raise MorseServerError('Unable to connect to the server. Check it is running and ' + \
                                 'that you provided the right host and port.')
        
        self.start()

        return

        # NOT DYNAMIC LOADING OF SERVICE YET

        #get the list of methods currenlty implemented by the server
        try:
            res = self.call_server(["listSimpleMethods"])
            self.rpc_methods = [(t.split('(')[0], len(t.split(','))) for t in res]
        except MorseServerError:
            self._morse_server.close()
            self.s.close()
            raise MorseServerError('Cannot initialize the morse connector! Smthg wrong with the server!')
        
        #add the the Morse class all the methods the server declares
        for m in self.rpc_methods:
            self.add_methods(m)

    def __enter__(self):
        return self


    def run(self):
        """ This method reads and writes to/from the simulator.
        When a new request is pushed in _morse_requests_queue, it is send to the 
        server, when the server answer smthg, we check if it's a normal answer
        or an event, and dispatch accordingly the answer's content.
        """
        
        inputs = [self.s]
        outputs = [self.s]
        
        while self._running:

        
            try:
                inputready,outputready,exceptready = select.select(inputs, outputs, [])
            except select.error as e:
                break
            except socket.error as e:
                break
            
            if not self._morse_requests_queue.empty():
                for o in outputready:
                    if o == self.s:
                        r = self._morse_requests_queue.get()
                        if r['args']:
                            r['args'] = ', '.join(json.dumps(a) for a in r['args'])
                        else:
                            r['args'] = ''
                        raw = "id{id} {component} {service} [{args}]\n".format(**r)
                        pymorselogger.debug("Sending " + raw)
                        self._morse_server.write(raw)
                        self._morse_server.flush()
            
            for i in inputready:
                if i == self.s:
                    res = self.get_morse_response()

                    # NOT USED YET
                    if res['status'] == "event": #notify the event
                        try:
                            evt_id = res['value'][0]
                            evt_params = res['value'][1:]
                            
                            cbThread = Thread(target=self._registered_events[evt_id], args=evt_params)
                            cbThread.start()
                            pymorselogger.debug("Event notified")
                            
                        except KeyError:
                            pymorselogger.error("Got a event notification, but I " + \
                            "don't know event " + evt_id)
                    else: #it's probably the answer to a request, push it forward.
                        self._morse_responses_queue.put(res)
            
            time.sleep(0.05)
    
       
    def get_morse_response(self):
        raw = self._morse_server.readline().rstrip('\n')

        result = None
        try:
            id, status, result = raw.split(' ', 2)

            try:
                result = json.loads(result)
            except TypeError:
                pymorselogger.error("Could not deserialize MORSE answer! Got: "+\
                        result)
        except ValueError:
            # No return value
            if ' ' in raw:
                id, status = raw.split(' ')
            else:
                pymorselogger.error("Could not receive a valid response from MORSE")
                pymorselogger.error("One of the tests has probably failed!")
                id = '???'
                status = 'FAIL'

        morse_answer = {"status": status,
                        "id": id,
                        "result":result}

        pymorselogger.debug("Got answer: " + morse_answer['status'] + \
                ", " + str(morse_answer['result']))

        return morse_answer

    def call_server(self, component, service, *args):
        """ Calls a service from the simulator.
        
        The call will block until a response from the simulator is received.
        
        :param component: the component that expose the service (like a robot name)
        :param service: the name of the service
        :param args...: (variadic) each service parameter, as a separate argument
        """
        
        req = {'id': self.id,
               'component': component,
               'service': service,
               'args': args}
        self.id += 1

        self._morse_requests_queue.put(req)
        
        res = self._morse_responses_queue.get() #block until we get an answer
        
        if res['status'] == SUCCESS:
            if not res['result']:
                return None
            return res['result']
            
        elif res['status'] == FAILURE:
            raise MorseServerError(res['result'])
        
        else:
            raise MorseServerError("Got an unexpected message status from MORSE: " + \
            res['status'])
        
    def add_methods(self, m):
        def innermethod(*args):
            req = ["%s" % m[0]]
            for a in args:
                req.append(str(a))
            pymorselogger.debug("Sending request: " + req[0])
            return self.call_server(req)
                
        innermethod.__doc__ = "This method is a proxy for the MORSE %s service." % m[0]
        innermethod.__name__ = m[0]
        setattr(self,innermethod.__name__,innermethod)
    
    def close(self):
        # Close data streams
        for stream in self.open_streams:
            stream.close()
        self.open_streams = []
        
        self._running = False
        self.join()
        pymorselogger.debug('Closing the connection to MORSE...')
        self._morse_server.close()
        self.s.close()
        pymorselogger.debug('Done. Bye bye!')
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    def robots(self):
       return self.call_server("simulation", "list_robots")

    def quit(self):
        self.call_server("simulation", "quit")
        self.close()

    def reset(self):
       return self.call_server("simulation", "reset")

    def streams(self):
       return self.call_server("simulation", "list_streams")

    def get_stream_port(self, stream):
       return self.call_server("simulation", "get_stream_port", stream)


    def stream(self, stream, maxlength = 100):

        port = self.get_stream_port(stream)

        #_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        #_client.connect((self.host, port))
        #pymorselogger.info("Connected to a MORSE stream (read only).")
        #return _client.makefile('r')

        stream =  MorseStream(self.host, port, readonly = True, maxlength = maxlength)
        self.open_streams.append(stream)

        return stream

if __name__ == '__main__':


    console = logging.StreamHandler()
    console.setLevel(4)

    # set a format which is simpler for console use
    formatter = logging.Formatter('%(asctime)-15s %(name)s: %(levelname)s - %(message)s')
    # tell the handler to use this format
    console.setFormatter(formatter)
    # add the handler to the root logger
    pymorselogger.addHandler(console)
    
    HOST = 'localhost'    # MORSE host
    PORT = 4000        # MORSE port
    
    pymorselogger.info("Starting now...")
    morse = Morse(HOST, PORT)
    
    try:
        print(str(morse.robots()))
        streams = morse.streams()
        print(str(streams))
        pose_stream = morse.stream(streams[0])

        print(pose_stream.get())

        morse.quit()

    except MorseServerError as ose:
        print('Oups! An error occured!')
        print(ose)
    finally:
        morse.close()
