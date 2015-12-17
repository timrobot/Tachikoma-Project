CC = gcc
INCLUDE = -I$(shell pwd)/../core
all: visual.o test.o visual

test.o: test.c
	$(CC) $(INCLUDE) -o $@ -c $<

visual: visual.o test.o
	$(CC) $(INCLUDE) -o $@ $^

visual.o: visual.c
	$(CC) $(INCLUDE) -o $@ -c $<

clean:
	rm -rf *.o visual
