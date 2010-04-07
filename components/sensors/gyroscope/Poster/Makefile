CC=gcc
LDFLAGS=-L${ROBOTPKG_BASE}/lib -lposterLib
CFLAGS=-I/usr/include/python2.6 -I${ROBOTPKG_BASE}/include -I${ROBOTPKG_BASE}/include/pom

# Is there a better way to know if we have an x86_64 kernel ?
ifneq (, $(filter x86_64, "$(shell uname -a)"))
	CFLAGS += -fPIC
endif

POSTER=poster
TARGET=$(POSTER)
STANDALONE=standalone

all:_$(TARGET).so $(STANDALONE)

$(TARGET).o: $(TARGET).c $(TARGET)_wrap.c
	@echo COMPILING
	$(CC) -c $(TARGET).c $(TARGET)_wrap.c $(CFLAGS)

_$(TARGET).so: $(TARGET).o
	@echo LINKING
	ld -shared -o _$(TARGET).so $(TARGET).o $(TARGET)_wrap.o $(LDFLAGS)

$(TARGET)_wrap.c: $(TARGET).i
	@echo SWIG
	swig -python $(TARGET).i

$(STANDALONE): $(STANDALONE).c
	@echo "COMPILING (STANDALONE C)"
#$(CC) -c $@.c $(CFLAGS)
#ld -o $@ $@.o $(LDFLAGS)
	$(CC) -g -o $@ $@.c $(CFLAGS) $(LDFLAGS)


.PHONY: clean
clean:
	-rm -f *.o _$(TARGET).so $(TARGET)_wrap.? $(TARGET).py $(TARGET).pyc
