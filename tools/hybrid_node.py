import json
import socket
import logging
import asynchat
import asyncore
import threading
# Double-ended queue, thread-safe append/pop.
from collections import deque
# initialize the logger
logger = logging.getLogger(__name__)
handler = logging.StreamHandler()
handler.setFormatter( logging.Formatter('[%(asctime)s][%(name)s][%(levelname)s] %(message)s') )
logger.addHandler( handler )
logger.setLevel(logging.INFO)
from lxml import etree

MSG_SEPARATOR=b"\n"

#
# XXX StreamJSON should be extern
#

class StreamB(asynchat.async_chat):
    """ Asynchrone I/O stream handler (raw bytes)

    To start the handler, just run :meth asyncore.loop: in a new thread::

    threading.Thread( target = asyncore.loop, kwargs = {'timeout': .1} ).start()

    where timeout is used with select.select / select.poll.poll.
    """

    use_encoding = 0 # Python2 compat.

    def __init__(self, host='localhost', port='1234', maxlen=100, sock=None):
        self.error = False
        asynchat.async_chat.__init__(self, sock=sock)
        if not sock:
            self.create_socket(family=socket.AF_INET, type=socket.SOCK_STREAM)
            self.connect( (host, port) )
        self.set_terminator(MSG_SEPARATOR)
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
        """ returns message as is (raw bytes) """
        return msg_bytes

    def encode(self, msg_bytes):
        """ returns message as is (raw bytes) plus the MSG_SEPARATOR """
        return msg_bytes + MSG_SEPARATOR


class Stream(StreamB):
    """ String Stream """
    def __init__(self, host='localhost', port='1234', maxlen=100, sock=None):
        StreamB.__init__(self, host, port, maxlen, sock)

    #### CODEC ####
    def decode(self, msg_bytes):
        """ decode bytes to string """
        return msg_bytes.decode()

    def encode(self, msg_str):
        """ encode string to bytes """
        return StreamB.encode(self, msg_str.encode())


class StreamJSON(Stream):
    """ JSON Stream """
    def __init__(self, host='localhost', port='1234', maxlen=100, sock=None):
        Stream.__init__(self, host, port, maxlen, sock)

    def decode(self, msg_bytes):
        """ decode bytes to json object """
        return json.loads(Stream.decode(self, msg_bytes))

    def encode(self, msg_obj):
        """ encode object to json string and then bytes """
        return Stream.encode(self, json.dumps(msg_obj))


#
# Real stuff starts here
#


robot_picoweb = {
    'mana'  : 'http://140.93.16.55:8080',
    'minnie': 'http://140.93.16.57:8080',
    #'mana'  : 'http://mana-superbase:8080',
    #'minnie': 'http://minnie-base:8080',
    #'target': 'http://chrome-dreams:8080/GPS?get=MEPos',
}

class Picoweb(object):
    def __init__(self, picoweb = "http://mana-superbase:8080"):
        self.picoweb = picoweb
        self.result = {
            'x': 0.0,
            'y': 0.0,
            'z': 0.0,
            'yaw':   0.0,
            'pitch': 0.0,
            'roll':  0.0,
        }
    def update(self):
        # get XML file from the robot's picoweb http server
        pom = etree.parse("%s/pom?get=Pos"%self.picoweb)
        # get the robot position in XML
        euler = pom.xpath("/pom/Pos/data/pomPos/mainToOrigin/euler").pop()
        def getf(key):
            return float(euler.xpath(key).pop().text)
        for key in self.result.keys():
            self.result[key] = getf(key)
        # return [(location), (rotation)] in XYZ, XYZ Euler
        return [ (self.result['x'], self.result['y'], self.result['z']), \
                 (self.result['roll'], self.result['pitch'], self.result['yaw']) ]

class HybridNode(object):
    """ Class definition for synchronisation of real robots with the MORSE simulator

    This component allows hybrid simulation, where the position of real robots
    is reflected in the simulator. Implemented using the socket multinode
    mechanism. The real robots are considered to be running in a node called
    "REAL", which reports the new positions to the multinode_server.
    """

    out_data = {}

    def __init__(self, host='localhost', port=65000):
        """ Create the socket that will be used to commmunicate to the server. """
        self.node_stream = None
        logger.debug("Connecting to %s:%d" % (host, port) )
        try:
            self.node_stream = StreamJSON(host, port)
            self.async_thread = threading.Thread( target = asyncore.loop, kwargs = {'timeout': .1} )
            self.async_thread.start()
            if self.node_stream.connected:
                logger.info("Connected to %s:%s" % (host, port) )
        except Exception as e:
            logger.info("Multi-node simulation not available!")
            logger.warning("Unable to connect to %s:%s"%(host, port) )
            logger.info(str(e))
        self.robots = {name: Picoweb(robot_picoweb[name]) for name in robot_picoweb.keys()}

    def _exchange_data(self):
        """ Send and receive pickled data through a socket """
        # Use the existing socket connection
        self.node_stream.publish(['hybrid', self.out_data])
        # Wait 1ms for incomming data or return the last one recieved
        return self.node_stream.get(timeout=.001) or self.node_stream.last()

    def synchronize(self):
        if not self.node_stream:
            logger.debug("not self.node_stream")
            return
        if not self.node_stream.connected:
            logger.debug("not self.node_stream.connected")
            return

        # Get the coordinates of local robots
        for robot,picow in self.robots.items():
            try:
                self.out_data[robot] = picow.update()
                logger.info(repr(self.out_data))
            except Exception as e:
                logger.error(str(e))

        # Send the encoded dictionary through a socket
        #  and receive a reply with any changes in the other nodes
        in_data = self._exchange_data()
        logger.debug("Received: %s" % repr(in_data))

    def __del__(self):
        """ Close the communication socket. """
        self.node_stream.close()
        # asyncore.close_all() # make sure all connection are closed
        self.async_thread.join(timeout=1)


def main(argv):
    if '-d' in argv[1:]:
        logger.setLevel(logging.DEBUG)

    logger.debug("Hybrid node started...")
    hn = HybridNode()

    try:
        while 1:
            hn.synchronize()
    except KeyboardInterrupt:
        logger.info("Quit (Ctrl+C)")
    finally:
        del hn
        logger.info("Closing all connections")
        asyncore.close_all()

    logger.info("Bye!")
    return 0

if __name__ == '__main__':
    import sys
    sys.exit(main(sys.argv))
