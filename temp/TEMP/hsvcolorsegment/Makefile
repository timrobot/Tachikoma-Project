CXX = g++
CFLAGS = -std=c++11 `pkg-config --cflags opencv`
LIBS = `pkg-config --libs opencv` -larmadillo \
			-L/usr/local/cuda-6.5/lib

all: videotest blob

blob: blob.o
	$(CXX) $(CFLAGS) -o $@ $^ $(LIBS)
test: test.o highgui.o
	$(CXX) $(CFLAGS) -o $@ $^ $(LIBS)
videotest: videotest.o highgui.o
	$(CXX) $(CFLAGS) -o $@ $^ $(LIBS)

%.o: %.cpp
	$(CXX) $(CFLAGS) -o $@ -c $<

clean:
	rm -f *.o test
