"""
from stream import Stream, PollThread
s = Stream('python.org', 80)
PollThread().start()
s.is_up()
s.publish("GET /\r\n")
s.get(.5) or s.last()
"""
import json
import socket
import logging
import asyncore
import asynchat
import threading
import traceback
# Double-ended queue, thread-safe append/pop.
from collections import deque

logger = logging.getLogger("pymorse")
logger.setLevel(logging.WARNING)
# logger.addHandler( logging.NullHandler() )

MSG_SEPARATOR=b"\n"

class PollThread(threading.Thread):
    def __init__(self, timeout=0.01):
        threading.Thread.__init__(self)
        self.keep_polling = True
        self.timeout = timeout
    def run(self):
        while asyncore.socket_map and self.keep_polling:
            asyncore.poll(self.timeout, asyncore.socket_map)
    def syncstop(self, timeout=None):
        self.keep_polling = False
        return self.join(timeout)
    #### with statement ####
    def __enter__(self):
        return self
    def __exit__(self, exc_type, exc_val, exc_tb):
        if not exc_type:
            self.syncstop(2 * self.timeout)
        else:
            self.syncstop(0)
            return False # re-raise exception

class StreamB(asynchat.async_chat):
    """ Asynchrone I/O stream handler (raw bytes)

    To start the handler, just run :meth asyncore.loop: in a new thread::

    threading.Thread( target = asyncore.loop, kwargs = {'timeout': .1} ).start()

    where timeout is used with select.select / select.poll.poll.
    """

    use_encoding = 0 # Python2 compat.

    def __init__(self, host='localhost', port=1234, maxlen=100, sock=None):
        self.error = False
        if not sock:
            sock = socket.socket(family=socket.AF_INET, type=socket.SOCK_STREAM)
            sock.connect( (host, port) )
        self._in_buffer  = b""
        self._in_queue   = deque([], maxlen)
        self._callbacks  = []
        self._cv_new_msg = threading.Condition()
        # init asynchat after connect and setting all locals avoids EBADF
        # and others undesirable effects of the asyncore.loop thread.
        asynchat.async_chat.__init__(self, sock=sock)
        self.set_terminator(MSG_SEPARATOR)

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
        logger.error('Exception occurred in asynchronous socket handler:\n%s'%traceback.format_exc())
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
        """ get the last message received

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
                    self.producer_fifo.extendleft([first, data])
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
    def __init__(self, host='localhost', port=1234, maxlen=100, sock=None):
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
    def __init__(self, host='localhost', port=1234, maxlen=100, sock=None):
        Stream.__init__(self, host, port, maxlen, sock)

    def decode(self, msg_bytes):
        """ decode bytes to json object """
        return json.loads(Stream.decode(self, msg_bytes))

    def encode(self, msg_obj):
        """ encode object to json string and then bytes """
        return Stream.encode(self, json.dumps(msg_obj))
