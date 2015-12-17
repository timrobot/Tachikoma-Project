CC = gcc
CFLAGS = -pedantic -Wall -Werror -fPIC -std=gnu99 -O3
OBJECTS = serial.o

all: $(OBJECTS)

serial.o: serial.c
	$(CC) $(CFLAGS) -o $@ -c $<

clean:
	rm -f $(OBJECTS)
