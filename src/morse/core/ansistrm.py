#
# Based on https://gist.github.com/758430
# Copyright (C) 2010-2011 Vinay Sajip. All rights reserved. Licensed under the new BSD license.
#
import logging
import os
from morse.helpers.morse_logging import SECTION, ENDSECTION

if os.name == 'nt':
    import ctypes
    import re

class ColorizingStreamHandler(logging.StreamHandler):
    # color names to indices
    color_map = {
        'black': 0,
        'red': 1,
        'green': 2,
        'yellow': 3,
        'blue': 4,
        'magenta': 5,
        'cyan': 6,
        'white': 7,
    }

    #levels to (background, foreground, bold/intense, blink -- only if bold = False)
    bright_scheme = {
        logging.DEBUG: (None, 'blue', False, False),
        logging.INFO: (None, 'white', False, False),
        logging.WARNING: (None, 'yellow', False, False),
        logging.ERROR: (None, 'red', False, False),
        logging.CRITICAL: ('red', 'white', True, False),
        SECTION: (None, 'green', True, False),
        ENDSECTION: (None, 'green', False, False),
    }
    
    dark_scheme = {
        logging.DEBUG: (None, 'blue', False, False),
        logging.INFO: (None, 'black', False, False),
        logging.WARNING: (None, 'yellow', False, False),
        logging.ERROR: (None, 'red', False, False),
        logging.CRITICAL: ('red', 'black', True, False),
        SECTION: (None, 'green', True, False),
        ENDSECTION: (None, 'green', False, False),
    }

    xmas_scheme = {
        logging.DEBUG: ('red', 'yellow', False, True),
        logging.INFO: ('red', 'white', False, True),
        logging.WARNING: ('red', 'yellow', False, True),
        logging.ERROR: ('red', 'yellow', False, True),
        logging.CRITICAL: ('red', 'white', False, True),
        SECTION: ('red', 'yellow', False, True),
        ENDSECTION: ('red', 'white', False, True),
    }
    
    csi = '\x1b['
    reset = '\x1b[0m'
    
    def __init__(self, scheme = None):
        super(ColorizingStreamHandler,self).__init__()
        if scheme == "xmas":
            self.level_map = self.xmas_scheme
        elif scheme == "dark":
            self.level_map = self.dark_scheme
        else:
            self.level_map = self.bright_scheme
    
    @property
    def is_tty(self):
        isatty = getattr(self.stream, 'isatty', None)
        return isatty and isatty()

    def emit(self, record):
        try:
            message = self.format(record)
            # Don't do anything if the StreamHandler does not exist
            if message == None:
                return
            stream = self.stream
            if not self.is_tty:
                stream.write(message)
            else:
                self.output_colorized(message)
            stream.write(getattr(self, 'terminator', '\n'))
            self.flush()
        except (KeyboardInterrupt, SystemExit):
            raise
        except:
            self.handleError(record)

    if os.name != 'nt':
        def output_colorized(self, message):
            self.stream.write(message)
    else:
        ansi_esc = re.compile(r'\x1b\[((?:\d+)(?:;(?:\d+))*)m')

        nt_color_map = {
            0: 0x00,    # black
            1: 0x04,    # red
            2: 0x02,    # green
            3: 0x06,    # yellow
            4: 0x01,    # blue
            5: 0x05,    # magenta
            6: 0x03,    # cyan
            7: 0x07,    # white
        }

        def output_colorized(self, message):
            parts = self.ansi_esc.split(message)
            write = self.stream.write
            h = None
            fd = getattr(self.stream, 'fileno', None)
            if fd is not None:
                fd = fd()
                if fd in (1, 2): # stdout or stderr
                    h = ctypes.windll.kernel32.GetStdHandle(-10 - fd)
            while parts:
                text = parts.pop(0)
                if text:
                    write(text)
                if parts:
                    params = parts.pop(0)
                    if h is not None:
                        params = [int(p) for p in params.split(';')]
                        color = 0
                        for p in params:
                            if 40 <= p <= 47:
                                color |= self.nt_color_map[p - 40] << 4
                            elif 30 <= p <= 37:
                                color |= self.nt_color_map[p - 30]
                            elif p == 1:
                                color |= 0x08 # foreground intensity on
                            elif p == 0: # reset to default color
                                color = 0x07
                            else:
                                pass # error condition ignored
                        ctypes.windll.kernel32.SetConsoleTextAttribute(h, color)

    def colorize(self, message, record):
        if record.levelno in self.level_map:
            bg, fg, bold, blink = self.level_map[record.levelno]
            params = []
            if bg in self.color_map:
                params.append(str(self.color_map[bg] + 40))
            if fg in self.color_map:
                params.append(str(self.color_map[fg] + 30))
            if bold:
                params.append('1')
            elif blink:
                params.append('5')
            if params:
                message = ''.join((self.csi, ';'.join(params),
                                   'm', message, self.reset))
        return message

    def format(self, record):
        try:
            message = logging.StreamHandler.format(self, record)
        # Catch the case when there is a zombie logger, when re-launching
        #  the simulation with 'p'.
        # This seems to be caused by an incorrect cleaning on the Builder
        except AttributeError as detail:
            return None
        if self.is_tty:
            message = self.colorize(message, record)
        return message

def main():
    root = logging.getLogger()
    root.setLevel(logging.DEBUG)
    root.addHandler(ColorizingStreamHandler())
    logging.debug('DEBUG')
    logging.info('INFO')
    logging.warning('WARNING')
    logging.error('ERROR')
    logging.critical('CRITICAL')

if __name__ == '__main__':
    main()

