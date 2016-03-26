CXX = g++
CFLAGS = -std=c++11 -O3 `pkg-config --cflags opencv`
VISUAL = $(shell pwd)/..
INCLUDE = -I/usr/local/include -I$(VISUAL)
LIBS = -L/usr/local/lib \
       `pkg-config --libs opencv` \
       -larmadillo
OBJS = $(VISUAL)/highgui.o $(VISUAL)/imgproc.o $(VISUAL)/draw.o $(VISUAL)/feature.o

all: visual_dir $(OBJS) test

visual_dir:
	make -C $(VISUAL)

test: $(OBJS) test.o
	$(CXX) $(CFLAGS) -o $@ $^ $(INCLUDE) $(LIBS)

%.o: %.cpp
	$(CXX) $(CFLAGS) -o $@ -c $< $(INCLUDE)

clean:
	rm -f *.o test
	make clean -C $(VISUAL)
