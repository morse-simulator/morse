# Makefile to compile control scripts

# Location of include libraries
OPENROBOTS_BASE = $(HOME)/openrobots

CC = g++

CFLAGS = -g -I$(OPENROBOTS_BASE)/include
LDFLAGS = -L$(OPENROBOTS_BASE)/lib -lYARP_OS -Wl,-rpath,$(OPENROBOTS_BASE)/lib

SCENE = scene_init-2.0

all: $(SCENE)

$(SCENE): $(SCENE).o
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^

%.o: %.cpp
	$(CC) $(CFLAGS) -c -o $@ $^

tags:
	ctags *c *.h

clean:
	-rm -f *.o

distclean: clean
	rm -f ${SCENE}

.PHONY: all clean tags
