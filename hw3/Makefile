

CC=g++

CFLAGS=-std=c++0x -c -Wall
LFLAGS= -lmdp-simulation -L.

all: test

test: test.o
	$(CC) test.o -o test $(LFLAGS)

test.o: test.cc
	$(CC) $(CFLAGS) test.cc

clean:
	rm *o test