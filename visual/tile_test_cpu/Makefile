CXX = g++
CFLAGS = -std=c++11 -I/usr/local/include
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
			 -lopencv_line_descriptor \
			 -lopencv_stitching \
			 -larmadillo
OBJS = tile_test.o highgui.o imgproc.o draw.o

all: tile_test

highgui.o: highgui.cpp
	$(CXX) $(CFLAGS) -o $@ -c $<

imgproc.o: imgproc.cpp
	$(CXX) $(CFLAGS) -o $@ -c $<

draw.o: draw.cpp
	$(CXX) $(CFLAGS) -o $@ -c $<

tile_test.o: tile_test.cpp
	$(CXX) $(CFLAGS) -o $@ -c $<

tile_test: $(OBJS)
	$(CXX) $(CFLAGS) -o $@ $^ $(LIBS)

clean:
	rm -f *.o tile_test

run:
	make clean; make; ./tile_test
