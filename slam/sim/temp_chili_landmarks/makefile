CPP = g++
CPPFLAGS = `pkg-config --cflags opencv` -std=c++11 -O3 -g
COMPILE = $(CPP) $(CPPFLAGS)
LIBS = `pkg-config --libs opencv` -lchilitags -lpthread

all: chilitags

chilitags: chili_landmarks.cpp
	$(COMPILE) -o $@ $< $(LIBS)

clean:
	rm -rfv chilitags