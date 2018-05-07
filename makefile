CFLAGS = -DSPI_DEV=\"/dev/spidev0.0\" -Wall -Winline -std=c99 -g
LDFLAGS = 

DEPS = pulutof.h
OBJ = main.o pulutof.o tcp_comm.o tcp_parser.o

all: main

%.o: %.c $(DEPS)
	gcc -c -o $@ $< $(CFLAGS) -pthread

main: $(OBJ)
	gcc $(LDFLAGS) -o rn1host $^ -lm -pthread

e:
	gedit --new-window main.c pulutof.h pulutof.c tcp_comm.c tcp_comm.h tcp_parser.c tcp_parser.h &
