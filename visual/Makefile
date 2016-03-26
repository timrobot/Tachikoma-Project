CXX = g++
CFLAGS = -std=c++11 -O3 `pkg-config --cflags opencv`
LIBS = -L/usr/local/lib \
			 `pkg-config --libs opencv` \
			 -larmadillo
OBJS = highgui.o imgproc.o feature.o draw.o

all: $(OBJS)

%.o: %.cpp
	$(CXX) $(CFLAGS) -o $@ -c $<

clean:
	rm -f $(OBJS)
