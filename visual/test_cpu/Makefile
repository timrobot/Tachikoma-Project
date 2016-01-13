CC = g++
CFLAGS = `pkg-config --cflags opencv` -std=c++11
LIBS = `pkg-config --libs opencv` -larmadillo
OBJS = highgui.o imgproc.o feature.o draw.o
TEST = test

all: $(OBJS) $(TEST)

test: $(OBJS) test.o
	$(CC) -o $@ $^ $(CFLAGS) $(LIBS)

feature.o: feature.cpp
	$(CC) $(CFLAGS) -o $@ -c $<

imgproc.o: imgproc.cpp
	$(CC) $(CFLAGS) -o $@ -c $<

highgui.o: highgui.cpp
	$(CC) $(CFLAGS) -o $@ -c $<

draw.o: draw.cpp
	$(CC) $(CFLAGS) -o $@ -c $<

test.o: test.cpp
	$(CC) $(CFLAGS) -o $@ -c $<

clean:
	rm -f $(OBJS) $(TEST)
