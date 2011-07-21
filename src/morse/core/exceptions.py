import logging; logger = logging.getLogger("morse." + __name__)

class MorseServiceError(Exception):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)

class MorseRPCInvokationError(MorseServiceError):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)

class MorseMethodNotFoundError(MorseRPCInvokationError):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)

class MorseWrongArgsError(MorseRPCInvokationError):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)
