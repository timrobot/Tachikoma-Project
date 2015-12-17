CC = g++
CFLAGS = `pkg-config --cflags opencv` -std=c++11
LIBS = `pkg-config --libs opencv`
OBJS = highgui.o test.o
TEST = test

all: $(OBJS) $(TEST)

test: $(OBJS)
	$(CC) -o $@ $^ $(CFLAGS) $(LIBS)

imgproc.o: imgproc.cpp
	$(CC) $(CFLAGS) -o $@ -c $<

highgui.o: highgui.cpp
	$(CC) $(CFLAGS) -o $@ -c $<

gcube.o: gcube.cpp
	$(CC) $(CFLAGS) -o $@ -c $<

test.o: test.cpp
	$(CC) $(CFLAGS) -o $@ -c $<

clean:
	rm -f $(OBJS) $(TEST)
