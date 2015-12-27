#CXX = g++
CXX = nvcc
CFLAGS = `pkg-config --cflags opencv` -std=c++11 -O3 -g
LIBS = `pkg-config --libs opencv` -larmadillo
OBJS = gcube.o highgui.o imgproc.o ovr.o test.o

all: $(OBJS) ovr

gcube.o: gcube.cu
	$(CXX) $(CFLAGS) -o $@ -c $<

highgui.o: highgui.cu
	$(CXX) $(CFLAGS) -o $@ -c $<

imgproc.o: imgproc.cu
	$(CXX) $(CFLAGS) -o $@ -c $<

ovr.o: ovr.cu
	$(CXX) $(CFLAGS) -o $@ -c $<

test.o: gpu_test.cpp
	$(CXX) $(CFLAGS) -o $@ -c $<

ovr: $(OBJS)
	$(CXX) $(CFLAGS) -o $@ $^ $(LIBS)

clean:
	rm -f $(OBJS) ovr
