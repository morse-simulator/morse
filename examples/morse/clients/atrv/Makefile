# Makefile to compile control scripts

# Location of include libraries
#ROBOTPKG_BASE = $(HOME)/openrobots

CC = g++

CFLAGS = -g -I$(ROBOTPKG_BASE)/include
LDFLAGS = -L$(ROBOTPKG_BASE)/lib -lYARP_init -lYARP_OS -Wl,-rpath,$(ROBOTPKG_BASE)/lib
POCOLIBSFLAGS = -lposterLib


MOTION = agv_motion-1.0
DESTINATION = atrv_destination-1.0
CAMERAS = atrv_dest_cams-1.0
CONTROL = atrv_status-1.0
GRID = grid_move-1.0
POCO_CLIENT = pocolibs_movement_client
PLATINE_CLIENT = pocolibs_platine_client

all: $(MOTION) $(DESTINATION) $(CAMERAS) $(CONTROL) $(GRID)

$(MOTION): $(MOTION).o
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^

$(DESTINATION): $(DESTINATION).o
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^

$(CAMERAS): $(CAMERAS).o
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^

$(CONTROL): $(CONTROL).o
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^

$(GRID): $(GRID).o
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^

%.o: %.cpp
	$(CC) $(CFLAGS) $(LDFLAGS) -c -o $@ $^

$(POCO_CLIENT): $(POCO_CLIENT).cpp
	@echo "COMPILING ($@ C)"
#$(CC) -c $@.c $(CFLAGS)
#ld -o $@ $@.o $(LDFLAGS)
	$(CC) -g -o $@ $^ $(CFLAGS) $(LDFLAGS) $(POCOLIBSFLAGS)

$(PLATINE_CLIENT): $(PLATINE_CLIENT).cpp
	@echo "COMPILING ($@ C)"
#$(CC) -c $@.c $(CFLAGS)
#ld -o $@ $@.o $(LDFLAGS)
	$(CC) -g -o $@ $^ $(CFLAGS) $(LDFLAGS) $(POCOLIBSFLAGS)


tags:
	ctags *c *.h

clean:
	-rm -f *.o

distclean:	clean
	rm -f ${MOTION} ${DESTINATION} ${CAMERAS} ${CONTROL}

.PHONY: all clean tags
