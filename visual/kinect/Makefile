CXX = g++
CCFLAGS = -fPIC -g -Wall -std=c++11
INCLUDE = -I/usr/local/include \
			-I/usr/local/include/libfreenect \
			-I/usr/include/libusb-1.0
LIBS = -L/usr/local/lib \
		-lfreenect \
		-larmadillo \
		-lopencv_core \
		-lopencv_imgproc \
		-lopencv_highgui \
		-pthread
OBJECTS = kinect.o
TEST = test.o test

all: $(OBJECTS)

kinect.o: kinect.cpp
	$(CXX) $(INCLUDE) $(CCFLAGS) -o $@ -c $<

test.o: test.cpp
	$(CXX) $(INCLUDE) $(CCFLAGS) -o $@ -c $<

test: test.o kinect.o
	$(CXX) $(INCLUDE) $(CCFLAGS) -o $@ $^ $(LIBS)

clean:
	rm -f $(OBJECTS) $(TEST)
