#!/usr/bin/python
"""A Python bridge to the ORO-server ontology server.

This library use the standard Python logging mechanism.
You can retrieve pymorse log messages through the "pymorse" logger. See the end of
this file for an example showing how to display to the console the log messages.
"""
import time
import logging
import socket
import select
from threading import Thread

try:
    from queue import Queue
except ImportError:
    # Python 2.x
    from Queue import Queue

import json

DEBUG_LEVEL=logging.DEBUG

class NullHandler(logging.Handler):
    """Defines a NullHandler for logging, in case pymorse is used in an application
    that doesn't use logging.
    """
    def emit(self, record):
        pass

pymorselogger = logging.getLogger("pymorse")

h = NullHandler()
pymorselogger.addHandler(h)

SUCCESS='SUCCESS'
FAILURE='FAILED'

class MorseServerError(Exception):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)

class MorseStream:

    def __init__(self, host, port, readonly = True):

        self._client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._client.connect((host, port))

        if readonly:
            self.stream = self._client.makefile('r')
            pymorselogger.info("Connected to a MORSE stream (read only).")
        else:
            self.stream = self._client.makefile('rw')
            pymorselogger.info("Connected to a MORSE stream (read and write).")

    def __del__(self):

        if self.stream:
            self.stream.close()

        if self._client:
            pymorselogger.info("Shutting down the socket...")
            self._client.shutdown(socket.SHUT_RDWR)
            pymorselogger.info("Closing socket client...")
            self._client.close()

    def pop(self):
        return json.loads(self.stream.readline())


class Morse(Thread):

    id = 0

    def __init__(self, host = "localhost", port = 4000):
        Thread.__init__(self)
        
        self._morse_requests_queue = Queue()
        self._morse_responses_queue = Queue()
        
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
    
    def run(self):
        """ This method reads and writes to/from the simulator.
        When a new request is pushed in _morse_requests_queue, it is send to the 
        server, when the server answer smthg, we check if it's a normal answer
        or an event, and dispatch accordingly the answer's content.
        """
        
        inputs = [self._morse_server]
        outputs = [self._morse_server]
        
        while self._running:
        
            try:
                inputready,outputready,exceptready = select.select(inputs, outputs, [])
            except select.error as e:
                break
            except socket.error as e:
                break
            
            if not self._morse_requests_queue.empty():
                for o in outputready:
                    if o == self._morse_server:
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
                if i == self._morse_server:
                    res = self.get_morse_response()

                    # NOT USED YET
                    if res['status'] == "event": #notify the event
                        try:
                            evt_id = res['value'][0]
                            evt_params = res['value'][1:]
                            
                            cbThread = Thread(target=self._registered_events[evt_id], args=evt_params)
                            cbThread.start()
                            pymorselogger.log(4, "Event notified")
                            
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
            id, status = raw.split(' ')

        morse_answer = {"status": status,
                        "id": id,
                        "result":result}

        pymorselogger.log(4, "Got answer: " + morse_answer['status'] + \
                ", " + str(morse_answer['result']))

        return morse_answer

    def call_server(self, component, service, *args):
        
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
            pymorselogger.log(4, "Sending request: " + req[0])
            return self.call_server(req)
                
        innermethod.__doc__ = "This method is a proxy for the MORSE %s service." % m[0]
        innermethod.__name__ = m[0]
        setattr(self,innermethod.__name__,innermethod)
    
    def close(self):
        self._running = False
        self.join()
        pymorselogger.log(4, 'Closing the connection to MORSE...')
        self._morse_server.close()
        self.s.close()
        pymorselogger.log(4, 'Done. Bye bye!')
    
    def __del__(self):
        if self._morse_server:
            self.close()

    def robots(self):
       return self.call_server("simulation", "list_robots")

    def quit(self):
       return self.call_server("simulation", "quit")

    def reset(self):
       return self.call_server("simulation", "reset")

    def streams(self):
       return self.call_server("simulation", "list_streams")

    def get_stream_port(self, stream):
       return self.call_server("simulation", "get_stream_port", stream)


    def join_stream(self, stream):

        port = self.get_stream_port(stream)

        #_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        #_client.connect((self.host, port))
        #pymorselogger.info("Connected to a MORSE stream (read only).")
        #return _client.makefile('r')

        return MorseStream(self.host, port, readonly = True)

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
    
    morse = Morse(HOST, PORT)
    
    pymorselogger.info("Starting now...")
    try:
        print(str(morse.robots()))
        streams = morse.streams()
        print(str(streams))
        pose_stream = morse.join_stream(streams[0])

        print(pose_stream.readline())

        morse.quit()

    except MorseServerError as ose:
        print('Oups! An error occured!')
        print(ose)
    finally:
        morse.close()
