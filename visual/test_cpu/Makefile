CC = g++
CFLAGS = `pkg-config --cflags opencv` -std=c++11 -g
LIBS = `pkg-config --libs opencv` -larmadillo
OBJS = highgui.o test.o imgproc.o
TEST = test

all: $(OBJS) $(TEST)

test: $(OBJS)
	$(CC) -o $@ $^ $(CFLAGS) $(LIBS)

imgproc.o: imgproc.cpp
	$(CC) $(CFLAGS) -o $@ -c $<

highgui.o: highgui.cpp
	$(CC) $(CFLAGS) -o $@ -c $<

test.o: test.cpp
	$(CC) $(CFLAGS) -o $@ -c $<

clean:
	rm -f $(OBJS) $(TEST)
