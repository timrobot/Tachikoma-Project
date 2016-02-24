CFLAGS = `pkg-config --cflags opencv` \
	`pkg-config --cflags gstreamer-1.0` -g
LIBS = `pkg-config --libs opencv` \
	`pkg-config --libs gstreamer-1.0`

TARGETS = test

all: $(TARGETS)

test: test.cpp
	g++ -o $@ $< $(CFLAGS) $(LIBS)

clean:
	rm -f $(TARGETS)
