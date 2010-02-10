CC=gcc
LDFLAGS=
CFLAGS=-I/usr/include/python2.6 -I/usr/lib/python2.6 -fPIC

CONVERT=convert
TARGET=$(CONVERT)

all:_$(CONVERT).so

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
