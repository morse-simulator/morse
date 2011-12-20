#!/bin/sh
# set the $OPENROBOTS_BASE variable to YARP_ROOT

g++ -g ressac_client.cpp -o ressac_client_charles -I $OPENROBOTS_BASE/include -l YARP_OS -L $OPENROBOTS_BASE/lib/ -Wl,-rpath,$OPENROBOTS_BASE/lib/
