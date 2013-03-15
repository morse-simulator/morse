class MorseError(Exception):
    """ General MORSE Error. """
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)

class MorseEnvironmentError(MorseError):
    """ Morse Error triggered while manipulating MORSE environments
    (typically, wrong permissions on a file or inexistant environment).
    """
    pass


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

class MorseBuilderError(MorseError):
    """ Morse Error caused by the Builder API.
    """
    pass

class MorseBuilderNoComponentError(MorseBuilderError):
    """ Morse Error caused by a wrong component in Builder.
    """
    def __init__(self, value):
        self.value = value
        import sys
        sys.exit("Unable to create simulation scene. Check builder script for typos.\nExecution terminated!")

class MorseBuilderBadSyntaxError(MorseBuilderError):
    """ Morse Error caused by a mistyped method or object name in Builder.
    """
    def __init__(self, value):
        self.value = value
        import sys
        sys.exit("Method or object name not found. Check builder script for typos.\nExectution terminated!")
