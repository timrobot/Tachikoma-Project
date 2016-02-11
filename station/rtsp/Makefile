CC = g++
CFLAGS = -pedantic -Wall -std=c++11 -O3
GSTFLAGS = `pkg-config --cflags gstreamer-1.0`
GSTLIBS = `pkg-config --libs gstreamer-1.0`
OPENCVFLAGS = `pkg-config --cflags opencv`
OPENCVLIBS = `pkg-config --libs opencv`
INCLUDE = -I/usr/local/include/gstreamer-1.0 -I/usr/include/glib-2.0

all: tcprecv.o tcprecv

tcprecv.o: tcprecv.cpp
	$(CC) $(CFLAGS) $(GSTFLAGS) $(OPENCVFLAGS) -o $@ -c $< $(INCLUDE)

tcprecv: tcprecv.o
	$(CC) $(CFLAGS) $(GSTFLAGS) $(OPENCVFLAGS) -o $@ $^ $(INCLUDE) $(GSTLIBS) $(OPENCVLIBS)

test.o: test.cpp
	$(CC) $(CFLAGS) $(GSTFLAGS) $(OPENCVFLAGS) -o $@ -c $< $(INCLUDE)

test: test.o
	$(CC) $(CFLAGS) $(GSTFLAGS) $(OPENCVFLAGS) -o $@ $^ $(INCLUDE) $(GSTLIBS) $(OPENCVLIBS)

test2.o: test2.cpp
	$(CC) $(CFLAGS) $(GSTFLAGS) $(OPENCVFLAGS) -o $@ -c $< $(INCLUDE)

test2: test2.o
	$(CC) $(CFLAGS) $(GSTFLAGS) $(OPENCVFLAGS) -o $@ $^ $(INCLUDE) $(GSTLIBS) $(OPENCVLIBS)

clean:
	rm -f test test2 tcprecv *.o
