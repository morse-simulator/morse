# Makefile to compile control scripts

# Location of include libraries
OPENROBOTS_BASE = $(HOME)/openrobots

CC = g++

CFLAGS = -g
LDFLAGS = -I$(OPENROBOTS_BASE)/include -L$(OPENROBOTS_BASE)/lib -lYARP_OS -Wl,-rpath,$(OPENROBOTS_BASE)/lib

SCENE = scene_init-2.0

all: $(ATRV)

$(SCENE): $(SCENE).o
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^

tags:
	ctags *c *.h

clean:
	-rm -f *.o

.PHONY: all clean tags
