CC = g++
CFLAGS = -Wall -O3
INCLUDE = -I/usr/include/festival -I/usr/include/speech_tools
LIBS = -L/usr/lib/festival -lFestival -lestools -lestbase -leststring
OBJECTS = tts.o
TEST = test.o test

all: $(OBJECTS)

tts.o: tts.cpp
	$(CC) $(CFLAGS) $(INCLUDE) -o $@ -c $<

test.o: test.cpp
	$(CC) $(CFLAGS) -o $@ -c $<

test: $(OBJECTS) test.o
	$(CC) $(CFLAGS) $(INCLUDE) -o $@ $^ $(LIBS)

clean:
	rm -f $(OBJECTS) $(TEST)
