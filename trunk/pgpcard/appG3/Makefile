# Variables
ARCH   := $(shell getconf LONG_BIT)
CC     := g++
CFLAGS := -Wall -m$(ARCH) -I$(PWD) -lpthread -lrt -lm

all:
	$(CC) $(CFLAGS) xStatus.cpp -o xStatus
	$(CC) $(CFLAGS) xDebug.cpp -o xDebug
	$(CC) $(CFLAGS) xReset.cpp -o xReset
	$(CC) $(CFLAGS) xLoop.cpp -o xLoop
	$(CC) $(CFLAGS) xWrite.cpp -o xWrite
	$(CC) $(CFLAGS) xRead.cpp -o xRead
	$(CC) $(CFLAGS) evrRunDelay.cpp -o evrRunDelay
	$(CC) $(CFLAGS) evrRunCode.cpp -o evrRunCode
	$(CC) $(CFLAGS) evrReset.cpp -o evrReset
	$(CC) $(CFLAGS) evrLaneMode.cpp -o evrLaneMode
	$(CC) $(CFLAGS) evrLaneEnable.cpp -o evrLaneEnable
	$(CC) $(CFLAGS) evrHdrChk.cpp -o evrHdrChk
	$(CC) $(CFLAGS) evrEnable.cpp -o evrEnable
	$(CC) $(CFLAGS) evrAcceptCode.cpp -o evrAcceptCode
	$(CC) $(CFLAGS) evrFiducial.cpp -o evrFiducial
	$(CC) $(CFLAGS) evrDisable.cpp -o evrDisable
	$(CC) $(CFLAGS) evrConfig.cpp -o evrConfig
	$(CC) $(CFLAGS) evrClearFrameCount.cpp -o evrClearFrameCount
	$(CC) $(CFLAGS) evrAcceptDelay.cpp -o evrAcceptDelay
	$(CC) -c $(CFLAGS) McsRead.cpp -o McsRead.o
	$(CC) -c $(CFLAGS) PgpCardG3Prom.cpp -o PgpCardG3Prom.o
	$(CC) $(CFLAGS) McsRead.o PgpCardG3Prom.o xPromLoad.cpp -o xPromLoad
	$(CC) $(CFLAGS) McsRead.o PgpCardG3Prom.o xPromVerify.cpp -o xPromVerify

clean:
	rm -f xStatus
	rm -f xReset
	rm -f xLoop
	rm -f xWrite
	rm -f xRead
	rm -f xRate
	rm -f McsRead.o
	rm -f PgpCardG3Prom.o
	rm -f xPromLoad
	rm -f xPromVerify
