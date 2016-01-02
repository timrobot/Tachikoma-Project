CC = nvcc
CXX = g++
CFLAGS = -std=c++11 -I/usr/local/include
LIBS = -L/usr/local/lib \
			 -lopencv_core \
			 -lopencv_highgui \
			 -lopencv_imgproc \
			 -lopencv_videoio \
			 -lopencv_imgcodecs
OBJS = highgui.o imgproc.o gcube.o gpu_util.o
TEST = test_centroid

all: $(OBJS) $(TEST)

test: test.o $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LIBS)

test.o: test.cpp
	$(CXX) $(CFLAGS) -o $@ -c $<

test_centroid: $(OBJS) centroid.o test_centroid.o
	$(CC) $(CFLAGS) -o $@ $^ $(LIBS)

test_centroid.o: test_centroid.cpp
	$(CC) $(CFLAGS) -o $@ -c $<

imgproc.o: imgproc.cu
	$(CC) $(CFLAGS) -o $@ -c $<

highgui.o: highgui.cu
	$(CC) $(CFLAGS) -o $@ -c $<

gcube.o: gcube.cu
	$(CC) $(CFLAGS) -o $@ -c $<

centroid.o: centroid.cu
	$(CC) $(CFLAGS) -o $@ -c $<

gpu_util.o: gpu_util.cu
	$(CC) $(CFLAGS) -o $@ -c $<

clean:
	rm -f *.o $(OBJS) $(TEST)
