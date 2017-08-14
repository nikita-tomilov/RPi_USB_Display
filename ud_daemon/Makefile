CFLAGS=-c -std=gnu99 -Wall

all: main

main: main.o 
	gcc main.o -o uddaemon -lm

main.o: main.c
	gcc $(CFLAGS) main.c

clean:
	rm -rf *.o uddaemon
