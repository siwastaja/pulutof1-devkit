CFLAGS = -DSPI_DEV=\"/dev/spidev0.0\" -Wall -Winline -Wno-int-conversion -Wno-unused-function -std=c99
LDFLAGS = 

DEPS = pulutof.h
OBJ = main.o pulutof.o tcp_comm.o tcp_parser.o

all: main spiprog

%.o: %.c $(DEPS)
	gcc -c -o $@ $< $(CFLAGS) -pthread

main: $(OBJ)
	gcc $(LDFLAGS) -o main $^ -lm -pthread

spiprog: spiprog.c
	gcc -o spiprog spiprog.c -std=c99 -Wno-int-conversion

e:
	gedit --new-window main.c pulutof.h pulutof.c tcp_comm.c tcp_comm.h tcp_parser.c tcp_parser.h &
