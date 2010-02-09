# Makefile to compile control scripts

# Location of include libraries
OPENROBOTS_BASE = $(HOME)/openrobots

CC = g++

CFLAGS = -g
LDFLAGS = -I$(OPENROBOTS_BASE)/include -L$(OPENROBOTS_BASE)/lib -lYARP_OS -Wl,-rpath,$(OPENROBOTS_BASE)/lib

RESSAC = ressac_client

all: $(RESSAC)

$(RESSAC): $(RESSAC).o
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^

%.o: %.cpp
	$(CC) $(CFLAGS) $(LDFLAGS) -c -o $@ $^

tags:
	ctags *c *.h

clean:
	-rm -f *.o

distclean: clean
	rm ${RESSAC}

.PHONY: all clean tags
