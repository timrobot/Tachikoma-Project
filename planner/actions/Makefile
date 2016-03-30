CXX = g++
CFLAGS = -std=c++11 -g `pkg-config --cflags opencv`
LIBS = -larmadillo -lSDL2 `pkg-config --libs opencv` -pthread
OBJS = actions.o

all: $(OBJS)

%.o: %.cpp
	$(CXX) $(CFLAGS) -o $@ -c $<

clean:
	rm -f *.o
