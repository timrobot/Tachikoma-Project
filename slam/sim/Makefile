CFLAGS = -I/usr/local/include \
<<<<<<< HEAD
				 -std=c++11 -O2 -g
LIBS = -L/usr/local/lib \
			 -lopencv_core \
			 -lopencv_highgui \
=======
				 -std=c++11 -O2
LIBS = -L/usr/local/lib \
			 -lopencv_core \
			 -lopencv_highgui \
			 -lopencv_imgcodecs \
>>>>>>> 319b19be6a74018905431f5a4ce3e74df54366d0
			 -larmadillo \
			 -lSDL2

OBJS = sim_window.o sim_map.o sim_robot.o sim_lidar.o sim_landmark.o \
			 pfilter.o \
			 highgui.o draw.o sim.o

all: $(OBJS) sim

raycast: raycast.cpp
	g++ -std=c++11 raycast.cpp -o raycast -larmadillo -lSDL2

sim: $(OBJS)
	g++ $(CFLAGS) -o $@ $^ $(LIBS)

%.o: %.cpp
	g++ $(CFLAGS) -o $@ -c $<

clean:
	rm -f raycast sim *.o
