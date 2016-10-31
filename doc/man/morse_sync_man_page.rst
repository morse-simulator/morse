:orphan:

morse_sync manual page
======================

Synopsis
--------

**morse_sync** [-h] [-d] [-H host] [-P port] [-t timeout]  -p <period> 

Description
-----------

**morse_sync** is a tool which sends a periodic signal to the Morse simulator,
allowing it to by synchronized with an external clock. It relies on Morse's socket
synchronisation *protocol*.

In addition, **morse_sync** can read a command in its input. It accepts the following
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
        Configure the port for Morse's synchronisation socket
:-p, --period <period>:
        Configure the period (in seconds) at which the signal is sent to Morse
:-h, --help:
        Displays information regarding the program use.
:-t, --timeout <timeout>:
        Configure the time (in seconds) that **morse_sync** should wait
        trying to connect to the Morse synchronisation socket. This is
        particularly useful for handling any delay in Morse's
        initialisation. The default value is 10 sec.

See Also
--------
:manpage:`morse(1)` 
