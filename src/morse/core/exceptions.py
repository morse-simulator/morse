class MorseError(Exception):
    """ General MORSE Error. """
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)

class MorseMiddlewareError(MorseError):
    """ Morse Error caused by a Middleware.
    """
    pass

class MorseMultinodeError(MorseError):
    """ Morse Error caused by a Multinode configuration.
    """
    pass

class MorseServiceError(MorseError):
    """ Morse Error caused by a Service.
    """
    pass

class MorseRPCInvokationError(MorseServiceError):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)

class MorseServiceAlreadyRunningError(MorseRPCInvokationError):
    def __init__(self, running_service, value):
        self.value = value
        self.running_service = running_service
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

class MorseRPCNbArgsError(MorseWrongArgsError):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)

class MorseRPCTypeError(MorseWrongArgsError):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)
