# Variables
ARCH   := $(shell getconf LONG_BIT)
CC     := g++
CFLAGS := -Wall -m$(ARCH) -I$(PWD) -lpthread -lrt -lm

all:
	$(CC) $(CFLAGS) setScratch.cpp -o setScratch
	$(CC) $(CFLAGS) xwrite.cpp -o xwrite
	$(CC) $(CFLAGS) xread.cpp -o xread
	$(CC) $(CFLAGS) xstatus.cpp -o xstatus
	$(CC) $(CFLAGS) xdebug.cpp -o xdebug
	$(CC) $(CFLAGS) xclearopenclients.cpp -o xclearopenclients
#	$(CC) $(CFLAGS) xcomp.cpp -o xcomp
	$(CC) $(CFLAGS) xreset.cpp -o xreset
	$(CC) $(CFLAGS) xloop.cpp -o xloop
#	$(CC) $(CFLAGS) xforward.cpp -o xforward
	$(CC) $(CFLAGS) xcntrst.cpp -o xcntrst
#	$(CC) $(CFLAGS) xrate.cpp -o xrate -lpthread
	$(CC) $(CFLAGS) xecho.cpp -o xecho
	$(CC) $(CFLAGS) xtrans.cpp -o xtrans
	$(CC) $(CFLAGS) xtesttx.cpp -o xtesttx
	$(CC) $(CFLAGS) xcopy.cpp -o xcopy -lrt
#	g++ -Wall -m64 -g Tests.cpp -fopenmp -o Tests
	$(CC) -c $(CFLAGS) McsRead.cpp -o McsRead.o
	$(CC) -c $(CFLAGS) PgpCardG2Prom.cpp -o PgpCardG2Prom.o
	$(CC) $(CFLAGS) McsRead.o PgpCardG2Prom.o xPromLoad.cpp -o xPromLoad
	$(CC) $(CFLAGS) McsRead.o PgpCardG2Prom.o xPromVerify.cpp -o xPromVerify

clean:
	rm -f setScratch
	rm -f xwrite
	rm -f xread
	rm -f xstatus
	rm -f xdebug
	rm -f xcomp
	rm -f xreset
	rm -f xloop
	rm -f xforward
	rm -f xcntrst
	rm -f xrate
	rm -f xecho
	rm -f xtrans
	rm -f xtesttx
	rm -f xcopy
	rm -f Tests
	rm -f McsRead.o
	rm -f PgpCardG2Prom.o
	rm -f xPromLoad
	rm -f xPromVerify
