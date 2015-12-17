CXX = g++
#CXX = nvcc
CFLAGS = `pkg-config --cflags opencv` -std=c++11 -O3
LIBS = `pkg-config --libs opencv` -larmadillo
OBJS = highgui.o imgproc.o ovr.o test.o

all: $(OBJS) ovr

gcube.o: gcube.cpp
	$(CXX) $(CFLAGS) -o $@ -c $<

highgui.o: highgui.cpp
	$(CXX) $(CFLAGS) -o $@ -c $<

imgproc.o: imgproc.cpp
	$(CXX) $(CFLAGS) -o $@ -c $<

ovr.o: ovr.cpp
	$(CXX) $(CFLAGS) -o $@ -c $<

test.o: test2.cpp
	$(CXX) $(CFLAGS) -o $@ -c $<

ovr: $(OBJS)
	$(CXX) $(CFLAGS) -o $@ $^ $(LIBS)

clean:
	rm -f $(OBJS) ovr
