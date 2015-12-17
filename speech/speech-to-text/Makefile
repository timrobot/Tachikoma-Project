CC = gcc
CFLAGS = -pedantic -Wall -std=gnu99 -O3
GSTFLAGS = `pkg-config --cflags gstreamer-0.10`
PSFLAGS = `pkg-config --cflags pocketsphinx sphinxbase`
GSTLIBS = `pkg-config --libs gstreamer-0.10`
PSLIBS = -L/usr/local/lib -lpocketsphinx -lsphinxbase -lsphinxad
#         `pkg-config --libs pocketsphinx sphinxbase`
INCLUDE = -I/usr/local/include/pocketsphinx/ -I/usr/local/include/sphinxbase/ \
          -I/usr/include/gstreamer-0.10/ -I/usr/include/glib-2.0/
MODELDIR = /usr/local/share/pocketsphinx/model
OBJECTS = pswrap.o rawrec.o listen.o stt.o
TARGET = listen
TEST = test.o test

all: $(OBJECTS) $(TARGET)

test: stt.o test.o
	$(CC) $(CFLAGS) -o $@ $^

test.o: test.c
	$(CC) $(CFLAGS) -o $@ -c $<

stt.o: stt.c
	$(CC) $(CFLAGS) -o $@ -c $< $(INCLUDE)

listen: pswrap.o rawrec.o listen.o
	$(CC) $(CFLAGS) $(PSFLAGS) $(GSTFLAGS) -o $@ $^ $(INCLUDE) \
	$(LIBS) $(PSLIBS) $(GSTLIBS) -DMODELDIR=\"$(MODELDIR)\"

listen.o: listen.c
	$(CC) $(CFLAGS) -o $@ -c $< $(INCLUDE)

pswrap.o: pswrap.c
	$(CC) $(CFLAGS) $(PSFLAGS) -o $@ -c $< $(INCLUDE) -DMODELDIR=\"$(MODELDIR)\"

rawrec.o: rawrec.c
	$(CC) $(CFLAGS) $(GSTFLAGS) -o $@ -c $< $(INCLUDE)

clean:
	rm -f $(TARGET) $(OBJECTS) $(TEST)
