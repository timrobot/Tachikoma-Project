CXX = g++
CFLAGS = -pedantic -Wall -Werror -fPIC --std=c++11 -O3
SERIAL_DIR = $(shell pwd)/serial
INCLUDE = -I$(SERIAL_DIR)
SUBDIRS = $(SERIAL_DIR)
LIBS = -larmadillo
OBJECTS = baserobot.o
LIBOBJS = $(SERIAL_DIR)/serial.o
TARGET = librobot.so

all: subdirs $(OBJECTS) $(TARGET)

subdirs:
	$(foreach directory, $(SUBDIRS), make -C $(directory);)

baserobot.o: baserobot.cpp
	$(CXX) $(CFLAGS) $(INCLUDE) -o $@ -c $<

librobot.so: $(OBJECTS) $(LIBOBJS)
	$(CXX) -shared -o $@ $^ $(LIBS)

clean:
	rm -f $(OBJECTS) $(TARGET)
	$(foreach directory, $(SUBDIRS), make clean -C $(directory);)
