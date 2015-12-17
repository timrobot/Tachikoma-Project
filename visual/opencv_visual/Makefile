CXX = g++
CFLAGS = `pkg-config --cflags opencv` -std=c++11 -I/usr/local/include
LIBS = -L/usr/local/lib \
			 -lopencv_core \
			 -lopencv_highgui \
			 -lopencv_imgproc \
			 -lopencv_videoio \
			 -lopencv_imgcodecs \
			 -lopencv_calib3d \
			 -lopencv_features2d \
			 -lopencv_xfeatures2d \
			 -lopencv_flann \
			 -lopencv_line_descriptor
OBJS = stereo.o

all: capture calibrate stereo

capture: capture.cpp
	$(CXX) $(CFLAGS) -o $@ $^ $(LIBS)

calibrate: calibrate.cpp
	$(CXX) $(CFLAGS) -o $@ $^ $(LIBS) -g

stereo.o: stereo.cpp
	$(CXX) $(CFLAGS) -o $@ -c $<

stereo: stereo.o
	$(CXX) $(CFLAGS) -o $@ $^ $(LIBS)

clean:
	rm -f *.o capture calibrate stereo *.pyc
