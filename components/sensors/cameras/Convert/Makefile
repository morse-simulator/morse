CC=gcc
LDFLAGS=
CFLAGS=-I/usr/include/python2.6 

# Is there a better way to know if we have an x86_64 kernel ?
ifneq (, $(filter x86_64, "$(shell uname -a))")
	CFLAGS += -fPIC
endif

CONVERT=convert
TARGET=$(CONVERT)

all:_$(TARGET).so

$(TARGET).o: $(TARGET).c $(TARGET)_wrap.c
	@echo COMPILING
	$(CC) -c $(TARGET).c $(TARGET)_wrap.c $(CFLAGS)

_$(TARGET).so: $(TARGET).o
	@echo LINKING
	ld -shared -o _$(TARGET).so $(TARGET).o $(TARGET)_wrap.o

$(TARGET)_wrap.c: $(TARGET).i
	@echo SWIG
	swig -python $(TARGET).i

.PHONY: clean

clean:
	-rm -f *.o _$(TARGET).so $(TARGET)_wrap.? $(TARGET).py $(TARGET).pyc
