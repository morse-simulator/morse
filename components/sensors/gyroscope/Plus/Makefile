CC=gcc
CCP=g++
LDFLAGS=-L${ROBOTPKG_BASE}/lib -lposterLib
CFLAGS=-I/usr/include/python2.6 -I${ROBOTPKG_BASE}/include -I${ROBOTPKG_BASE}/include/pom -I./

# Is there a better way to know if we have an x86_64 kernel ?
ifneq (, $(filter x86_64, "$(shell uname -a)"))
	CFLAGS += -fPIC
endif

GYRO_POSTER=gyro_poster
TARGET=$(GYRO_POSTER)
STANDALONE=standalone

all:_$(TARGET).so

$(TARGET).o: $(TARGET).cc $(TARGET)_wrap.cxx
	@echo COMPILING
	$(CCP) -c $^ $(CFLAGS)

_$(TARGET).so: $(TARGET).o $(TARGET)_wrap.o
	@echo LINKING
	ld -shared -o $@ $^ $(LDFLAGS)

$(TARGET)_wrap.cxx: $(TARGET).i $(TARGET).hh
	@echo SWIG
	swig -c++ -python $(TARGET).i

$(STANDALONE): $(STANDALONE).c
	@echo "COMPILING ($@ C)"
#$(CC) -c $@.c $(CFLAGS)
#ld -o $@ $@.o $(LDFLAGS)
	$(CC) -g -o $@ $@.c $(CFLAGS) $(LDFLAGS)

.PHONY: clean
clean:
	-rm -f *.o _$(TARGET).so $(TARGET)_wrap.* $(TARGET).py $(TARGET).pyc
