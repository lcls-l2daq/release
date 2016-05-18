#include <sys/types.h>
#include <linux/types.h>
#include <unistd.h>
#include <stdio.h>
#include <termios.h>
#include <fcntl.h>
#include <sstream>
#include <string>
#include <iomanip>
#include <iostream>
#include "../include/PgpCardMod.h"
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <math.h>
using namespace std;

#define DEVNAME "/dev/pgpcard"
#define MAXBYTES (1<<21)

long long int timeDiff(timespec* end, timespec* start) {
  long long int diff;
  diff =  (end->tv_sec - start->tv_sec) * 1000000000LL;
  diff += end->tv_nsec;
  diff -= start->tv_nsec;
  return diff;
}

int main (int argc, char **argv) {
  int           s;
  int           ret;

  int           count = 0;
  PgpCardTx     pgpCardTx;
  PgpCardRx     pgpCardRx;
  unsigned      eofeCount = 0;
  unsigned      fifoErrCount = 0;
  unsigned      lengthErrCount = 0;
  unsigned      mySize = 500000;
  long long int interval;
  timespec      sleepTime, remainingTime, myStart, myEnd;
  sleepTime.tv_sec = 0;
  sleepTime.tv_nsec = 0;

  s = open(DEVNAME, O_RDWR);

  pgpCardRx.maxSize = MAXBYTES/sizeof(unsigned);
  pgpCardRx.data = (uint *)malloc(MAXBYTES);
  pgpCardRx.model = sizeof(&pgpCardRx);

  pgpCardTx.pgpLane = pgpCardTx.pgpVc = 0;
  pgpCardTx.size    = mySize;
  pgpCardTx.data    = (uint *)malloc(mySize*sizeof(unsigned));
  pgpCardTx.model   = sizeof(&pgpCardTx);
  pgpCardTx.cmd     = IOCTL_Normal_Write;
  if (argc > 1) {
    ret = write (s, &pgpCardTx, sizeof(pgpCardTx));
    if ( ret <= 0 ) {
      perror("Write Error on seed write");
    }
  }

  clock_gettime(CLOCK_REALTIME, &myStart);

  do {

    ret = read(s,&pgpCardRx,sizeof(PgpCardRx));

    nanosleep(&sleepTime, &remainingTime);

    if ( ret > 0 ) {
      if (argc > 1) {
        if (pgpCardRx.rxSize != mySize) {
          printf("got size %u, count(%u)\n", pgpCardRx.rxSize, count);
        }
      }
      pgpCardTx.pgpLane = pgpCardRx.pgpLane;
      pgpCardTx.pgpVc = pgpCardRx.pgpVc;
      pgpCardTx.size = pgpCardRx.rxSize;
      pgpCardTx.data = pgpCardRx.data;
      ret = write (s,&pgpCardTx,sizeof(PgpCardTx));
      count++;
      if (pgpCardRx.eofe) eofeCount++;
      if (pgpCardRx.fifoErr) fifoErrCount++;
      if (pgpCardRx.lengthErr) lengthErrCount++;
      if (!(count%1000)) {
        if (1) {
          clock_gettime(CLOCK_REALTIME, &myEnd);
          interval = timeDiff(&myEnd, &myStart);
          float rate = (mySize*32.0*1000.0/interval);
          myStart.tv_nsec = myEnd.tv_nsec;
          myStart.tv_sec  = myEnd.tv_sec;
          printf("period(%6.3f sec), rate(%5.3f Gb/s), ", interval/1e9, rate);
        }
        printf("count(%u), eofe(%u), fifoErr(%u), lengthErr(%u)\n",
            count, eofeCount, fifoErrCount, lengthErrCount);
      }
    }

  } while (1);
  close(s);
}
