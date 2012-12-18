
import logging; pymorselogger = logging.getLogger("pymorse")

from threading import Lock

try:
    from queue import LifoQueue, Queue
except ImportError:
    # Python 2.x
    from Queue import LifoQueue, Queue

# Double-ended queue, thread-safe append/pop.
from collections import deque

import json

class Stream():

    def __init__(self, com, port, maxlength = 100):

        self.maxlength = maxlength
        self._ring_buffer = deque([], maxlength)
        self._in_queue = LifoQueue()
        self._out_queue = Queue()

        self.com = com

        self.port = port
        # First, connect to the stream port
        self.com.connect(port,
                         self._in_queue,
                         self._out_queue,
                         self._ring_buffer,
                         self.on_data)

        self.lock = Lock() # lock for concurrent access to self.subscribers
        self.subscribers = []

    def last(self, n = None):
        """ Returns the latest or the n latest data records read
        from the data stream.

        If called without parameter, returns a single record, as a
        Python object. If no record has been received yet, returns None.

        If called with a parameter n, returns the n latest ones or less
        if less records have been received (in older to most recent order).

        Returns None only if no data has been ever published on the stream.
        """

        # No data yet?
        if not self._ring_buffer:
            return None

        if not n:
            res = self._ring_buffer[0]
        else:
            n = min(n, self.maxlength)
            res = list(self._ring_buffer)[:n] #TODO: optimize that! currently, the whole queue is copied :-/
            res.reverse()

        return res

    def get(self, timeout = None):
        """ Blocks until a record is available from the data stream, and returns it.

        If records are already available, returns the most recent one.

        :param timeout: (default: None) If timeout is a positive number, it
        blocks at most timeout seconds and raises the Empty exception if no
        item was available within that time. If timeout is None, block
        indefinitely.

        """
        # in_queue is a LIFO: get() returns the last inserted element,
        # ie the most recent record.
        res = self._in_queue.get(True, timeout)

        # Clear the queue: we do not need to stack older result.
        pymorselogger.debug("Clearing stream incoming queue")
        with self._in_queue.mutex:
            self._in_queue.queue = []

        # Note that we may loose one records between the get() and the clear()
        # if the other thread reacquires the mutex. It is however not a big
        # issue to loose a record here.

        return res

    def on_data(self, data):
        # Since this method can takes some time (invoking all callback
        # on the new incoming data), we copy the list of callbacks
        # to release quickly the lock on self.subscribers
        with self.lock:
            cbs = self.subscribers[:]

        for cb in cbs:
            cb(data)

    def subscribe(self, cb):
        with self.lock:
            self.subscribers.append(cb)

    def unsubscribe(self, cb):
        with self.lock:
            self.subscribers.remove(cb)

    def publish(self, msg):
        self._out_queue.put(msg)

