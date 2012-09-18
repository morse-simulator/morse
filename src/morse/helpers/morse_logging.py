import logging
import time

starttime = time.time()

SECTION = 25 #INFO = 20, WARNING = 30
logging.addLevelName('SECTION', SECTION)

ENDSECTION = 23 #INFO = 20, WARNING = 30
logging.addLevelName('ENDSECTION', ENDSECTION)

class MorseFormatter(logging.Formatter):
    def __init__(self, *args, **kwargs):
        # can't do super(...) here because Formatter is an
        # old school class
        logging.Formatter.__init__(self, *args, **kwargs)

    def format(self, record):
        level = record.levelname
        module = ".".join(record.name.split(".")[-2:])
        message = logging.Formatter.format(self, record)

        if module not in ["morse", "morse.main", "blender.main"]:
            message = "[" + module + "] " + message

        if level == 'SECTION':
            message = "\n[" + message + "]"

        if level == 'ENDSECTION':
            message = "[" + message + "]"

        if level in ['DEBUG', 'INFO', 'WARNING']:
            message = "[{0: 9.3f}] ".format(record.created - starttime) + message

        return message
