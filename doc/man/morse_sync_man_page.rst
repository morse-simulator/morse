:orphan:

morse_sync manual page
======================

Synopsis
--------

**morse_sync** [-h] [-d] [-H host] [-P port] [-t timeout]  -p <period> 

Description
-----------

**morse_sync** is a tool which sends a periodic signal to the Morse simulator,
allowing to synchronize it to an external clock. It relies on the socket
synchronisation *protocol* of Morse.

Moreover, **morse_sync** reads command on its input. It accepts the following
commands:

:quit:       
        End the **morse_sync** tool
:set_period <period>:
        Change the period of message sent

Options
-------

:-d, --debug: 
        Enable debug information
:-H, --host <host>:
        Configure the host where Morse is running
:-P, --port <port>:
        Configure the port of the synchronisation socket of Morse
:-p, --period <period>:
        Configure the period (in sec) to which the signal is sent to Morse
:-h, --help:
        Displays information regarding the program use.
:-t, --timeout <timeout>:
        Configure time (in sec) during **morse_sync** tries to connect to the
        Morse synchronisation socket. It is particularly useful to handle
        delay in the Morse initialisation. The default value is 10 sec.

See Also
--------
:manpage:`morse(1)` 
