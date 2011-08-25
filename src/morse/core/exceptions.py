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
