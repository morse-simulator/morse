"""
Statistics helper for Morse usage
"""

class Stats:
    """
    A simple class to compute basic statistics on some values, computed
    incrementaly
    """
    def __init__(self):
        self.n = 0
        self.mean = 0.0
        self.m2 = 0.0

    def update(self, x):
        self.n = self.n + 1
        delta = x - self.mean
        self.mean = self.mean + delta/self.n
        self.m2 = self.m2 + delta * (x - self.mean)

    @property
    def variance(self):
        return self.m2 / (self.n - 1)
