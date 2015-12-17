CC = g++
CFLAGS = -pedantic -Wall -Werror -fPIC --std=c++11 -O3
SERIAL_DIR = $(shell pwd)/serial
SLAMBOT_DIR = $(shell pwd)/slambot
TACHIKOMA_DIR = $(shell pwd)/tachikoma
INCLUDE = -I$(SERIAL_DIR) \
          -I$(TACHIKOMA_DIR)
SUBDIRS = $(SERIAL_DIR) \
					$(TACHIKOMA_DIR)
LIBS = -larmadillo
OBJECTS = baserobot.o
LIBOBJS = $(SERIAL_DIR)/serial.o \
          $(TACHIKOMA_DIR)/tachikoma.o
TARGET = librobot.so

all: subdirs $(OBJECTS) $(TARGET)

subdirs:
	$(foreach directory, $(SUBDIRS), make -C $(directory);)

baserobot.o: baserobot.cpp
	$(CC) $(CFLAGS) $(INCLUDE) -o $@ -c $<

librobot.so: $(OBJECTS) $(LIBOBJS)
	$(CC) -shared -o $@ $^ $(LIBS)

clean:
	rm -f $(OBJECTS) $(TARGET)
	$(foreach directory, $(SUBDIRS), make clean -C $(directory);)
