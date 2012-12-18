from concurrent.futures import ThreadPoolExecutor, Future

class MorseFuture():
    def __init__(self, future):
        self.done = future.done
        self.running = future.running
        self.result = future.result
        self.exception = future.exception
        self.add_done_callback = future.add_done_callback

    def cancel(self):
        print("Cancelling future: TDB")

    def __lt__(self, other):
        """ Overrides the comparision operator (used by ==, !=, <, >) to
        first wait for the result of the future.
        """
        return self.result().__lt__(other)

    def __le__(self, other):
        return self.result().__le__(other)

    def __eq__(self, other):
        return self.result().__eq__(other)

    def __ne__(self, other):
        return self.result().__ne__(other)

    def __gt__(self, other):
        return self.result().__gt__(other)

    def __ge__(self, other):
        return self.result().__ge__(other)

    def __repr__(self):
        """ Overrides the representation function to 
        first wait for the result of the future.
        """
        return self.result().__repr__()

class MorseExecutor(ThreadPoolExecutor):

    def submit(self, fn, *args, **kwargs):
        f = super(MorseExecutor, self).submit(fn, *args, **kwargs)

        return MorseFuture(f)


