CC = g++
CFLAGS = -pedantic -Wall -Werror -fPIC -std=c++11 -O3
LIBS = -larmadillo
SERIAL = $(shell pwd)/../serial
BASEROBOT = $(shell pwd)/..
INCLUDE = -I$(BASEROBOT) -I$(SERIAL)
SUBDIRS = $(SERIAL)
SUBOBJS = $(SERIAL)/serial.o
OBJECTS = slambot.o xboxctrl.o
TEST = test.o test

all: subdirs $(OBJECTS)

subdirs:
	make -C $(BASEROBOT) baserobot.o
	$(foreach directory, $(SUBDIRS), make -C $(directory);)

slambot.o: slambot.cpp
	$(CC) $(CFLAGS) $(INCLUDE) -o $@ -c $<

xboxctrl.o: xboxctrl.c
	gcc -pedantic -Wall -Werror -std=gnu99 -O3 -c $< -o $@

test.o: test.cpp
	$(CC) $(CFLAGS) $(INCLUDE) -o $@ -c $<

test: $(BASEROBOT)/baserobot.o $(OBJECTS) $(SUBOBJS) xboxctrl.o test.o
	$(CC) $(CFLAGS) -pthread $(INCLUDE) -o $@ $^ $(LIBS)

clean:
	rm -f $(OBJECTS) xboxctrl.o $(TEST)
	$(foreach directory, $(SUBDIRS), make clean -C $(directory);)
