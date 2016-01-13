CC = nvcc
CFLAGS = -std=c++11 -I/usr/local/include
LIBS = -L/usr/local/lib \
			 -lopencv_core \
			 -lopencv_highgui \
			 -lopencv_imgproc \
			 -lopencv_videoio \
			 -lopencv_imgcodecs \
			 -larmadillo
OBJS = gpu_util.o gcube.o imgproc.o highgui.o
TEST = test

all: $(OBJS) $(TEST)

test: centroid.o test.o $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LIBS)

test.o: test.cpp
	$(CC) $(CFLAGS) -o $@ -c $<

centroid.o: centroid.cu
	$(CC) $(CFLAGS) -o $@ -c $<

imgproc.o: imgproc.cu
	$(CC) $(CFLAGS) -o $@ -c $<

highgui.o: highgui.cu
	$(CC) $(CFLAGS) -o $@ -c $<

gcube.o: gcube.cu
	$(CC) $(CFLAGS) -o $@ -c $<

gpu_util.o: gpu_util.cu
	$(CC) $(CFLAGS) -o $@ -c $<

clean:
	rm -f *.o $(OBJS) $(TEST)
