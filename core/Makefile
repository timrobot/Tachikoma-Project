CC = g++
CFLAGS = -pedantic -Wall -O3
INCLUDE = -I$(shell pwd) \
          -I$(shell pwd)/../user \
          -I$(shell pwd)/../agent \
          -I$(shell pwd)/../robot
LIBS = -L$(shell pwd)/../user -luser \
       -L$(shell pwd)/../agent -lagent \
	     -L$(shell pwd)/../robot -lrobot \
       -lFestival -lestools -lestbase -leststring \
       `pkg-config --libs opencv`
#       -larmadillo
SUBDIRS = $(shell pwd)/../user \
          $(shell pwd)/../agent \
          $(shell pwd)/../robot
OBJECTS = core.o manual.o
TARGET = core manual

all: subdirs $(OBJECTS) $(TARGET)

subdirs:
	$(foreach directory, $(SUBDIRS), make -C $(directory);)

core.o: core.cpp
	$(CC) $(CFLAGS) $(INCLUDE) -o $@ -c $<

core: core.o
	$(CC) $(CFLAGS) $(INCLUDE) -o $@ $^ $(LIBS)

manual.o: manual.cpp
	$(CC) $(CFLAGS) $(INCLUDE) -o $@ -c $<

manual: manual.o
	$(CC) $(CFLAGS) $(INCLUDE) -o $@ $^ $(LIBS)

clean:
	rm -f $(OBJECTS) $(TARGET) *.o dict
	$(foreach directory, $(SUBDIRS), make clean -C $(directory);)

dict.o: dict.c
	gcc -g -c $<

test.o: test.c
	gcc -g -c $<

dict: dict.o test.o
	gcc -g -o $@ $^
