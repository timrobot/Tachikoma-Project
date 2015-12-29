CC = nvcc
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
OBJS = highgui.o imgproc.o gcube.o
TEST = test test_centroid

all: $(OBJS) $(TEST)

test: test.o $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LIBS)

test.o: test.cpp
	$(CXX) $(CFLAGS) -o $@ -c $<

test_centroid: centroid.o $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LIBS)

test_centroid.o: test_centroid.cpp
	$(CXX) $(CFLAGS) -o $@ -c $<

imgproc.o: imgproc.cu
	$(CC) $(CFLAGS) -o $@ -c $<

highgui.o: highgui.cu
	$(CC) $(CFLAGS) -o $@ -c $<

gcube.o: gcube.cu
	$(CC) $(CFLAGS) -o $@ -c $<

clean:
	rm -f *.o $(OBJS) $(TEST)
