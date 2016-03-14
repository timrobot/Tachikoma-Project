CFLAGS = -std=c++11 -g `pkg-config --cflags opencv`
LIBS = -larmadillo -lSDL2 `pkg-config --libs opencv`
OBJS = astar.o maze_imgio.o sim.o maze_gen.o sim_window.o draw.o 

all: $(OBJS) sim

sim: $(OBJS)
	g++ $(CFLAGS) -o $@ $^ $(LIBS)

%.o: %.cpp
	g++ $(CFLAGS) -o $@ -c $<

clean:
	rm -f *.o sim
