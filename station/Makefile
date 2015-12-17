CC = gcc
CFLAGS = -pedantic -Wall -Werror -std=gnu99 -O3
INCLUDE = -I$(shell pwd)/../core
OBJECTS = user.o httplink.o xboxctrl.o
TARGET = libuser.a

all: $(OBJECTS) $(TARGET)

httplink.o: httplink.c
	$(CC) $(CFLAGS) -o $@ -c $<

xboxctrl.o: xboxctrl.c
	$(CC) $(CFLAGS) -o $@ -c $<

user.o: user.c
	$(CC) $(CFLAGS) -o $@ -c $< $(INCLUDE)

libuser.a: $(OBJECTS)
	ar rvs $@ $^

test.o: test.c
	$(CC) -o $@ -c $< $(INCLUDE)

test: xboxctrl.o test.o
	$(CC) $(CFLAGS) -o $@ $^

clean:
	rm -f $(OBJECTS) $(TARGET) $(TEST)
