CXX = g++
CFLAGS = -std=c++11 -g `pkg-config --cflags opencv`
LIBS = -larmadillo -lSDL2 `pkg-config --libs opencv` -pthread
ACTIONS = $(shell pwd)/../actions
INLCUDE = -I$(ACTIONS)
OBJS = astar.o maze_imgio.o sim.o maze_gen.o sim_window.o draw.o
SUBOBJS = $(ACTIONS)/actions.o

all: subdirs $(OBJS) sim

subdirs:
	make -C $(ACTIONS)

sim: $(OBJS)
	$(CXX) $(CFLAGS) -o $@ $^ $(LIBS)

%.o: %.cpp
	$(CXX) $(CFLAGS) -o $@ -c $<

clean:
	rm -f *.o sim
	make clean -C $(ACTIONS)
