CXX = g++
CFLAGS = -std=c++11 `pkg-config --cflags opencv`
LIBS = `pkg-config --libs opencv` -larmadillo

all: test blob

blob: blob.o
	$(CXX) $(CFLAGS) -o $@ $^ $(LIBS)
test: test.o highgui.o
	$(CXX) $(CFLAGS) -o $@ $^ $(LIBS)

%.o: %.cpp
	$(CXX) $(CFLAGS) -o $@ -c $<

clean:
	rm -f *.o test
