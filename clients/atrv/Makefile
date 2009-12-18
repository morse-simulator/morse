# Makefile to compile control scripts

# Location of include libraries
OPENROBOTS_BASE = $(HOME)/openrobots

CC = g++

CFLAGS = -g
LDFLAGS = -I$(OPENROBOTS_BASE)/include -L$(OPENROBOTS_BASE)/lib -lYARP_OS -Wl,-rpath,$(OPENROBOTS_BASE)/lib

MOTION = agv_motion-1.0
DESTINATION = atrv_destination-1.0
CAMERAS = atrv_dest_cams-1.0

all: $(MOTION) $(DESTINATION) $(CAMERAS)

$(MOTION): $(MOTION).o
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^

$(DESTINATION): $(DESTINATION).o
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^

$(CAMERAS): $(CAMERAS).o
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^

tags:
	ctags *c *.h

clean:
	-rm -f *.o

.PHONY: all clean tags
