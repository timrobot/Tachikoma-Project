CC = gcc
CFLAGS = -pedantic -std=gnu99 -O2
CXX = g++
CXXFLAGS = -pedantic -std=c++11 -O2 \
					 -I/usr/local/include \
					 `pkg-config --cflags opencv`
LIBS = `pkg-config --libs opencv` \
			 -L/usr/local/lib -lSDL \
			 -larmadillo \
			 -pthread \
			 -lchilitags

OBJS = serial.o \
			 baserobot.o \
			 actions.o \
			 astar.o \
			 chili_landmarks.o \
			 draw.o \
			 heap.o \
			 highgui.o \
			 mathfun.o \
			 pfilter.o \
			 tachikoma.o \
			 runrobot.o \
			 sim_landmark.o \
			 sim_map.o \
			 sim_robot.o

all: $(OBJS) runrobot

runrobot: $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LIBS)

%.o: %.c
	$(CC) $(CFLAGS) -o $@ -c $<

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -o $@ -c $<

clean:
	rm -rfv *.o runrobot
