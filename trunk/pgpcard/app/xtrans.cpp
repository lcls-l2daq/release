#include <sys/types.h>
#include <sys/ioctl.h>
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
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <math.h>
#include <signal.h>
using namespace std;

#define DEVNAME "/dev/pgpcard"

long long int timeDiff(timespec* end, timespec* start) {
  long long int diff;
  diff =  (end->tv_sec - start->tv_sec) * 1000000000LL;
  diff += end->tv_nsec;
  diff -= start->tv_nsec;
  return diff;
}

static bool quit = false;

enum {MagicNumber=0, QuadNumber=1, FrameCount=2, MAGIC=0xFEEDFACE};
unsigned      myEventCount = 0;

void sigfunc(int sig_no) {
  quit = true;
}

void printPgpCardTx(PgpCardTx* p) {
  printf("model(%u), cmd(%u), lane(%u), vc(%u), size(%u), quad(%u), count(%u)\n",
      p->model, p->cmd, p->pgpLane, p->pgpVc, p->size, p->data[QuadNumber], p->data[FrameCount]);
}

int main (int argc, char **argv) {
  int           s;
  int           ret;
  PgpCardTx     pgpCardTx[4];
  unsigned      mySize = 287129;
  long long int interval;
  timespec      sleepTime, remainingTime, myStart, myEnd;
  sleepTime.tv_sec = 0;
  sleepTime.tv_nsec = 0;
  (void)        signal(SIGINT, sigfunc);

  s = open(DEVNAME, O_RDWR);
  if (s<0) {
    perror("Opening pgp driver ");
    return -1;
  }

  for (unsigned i=0; i<4; i++ ) {
    pgpCardTx[i].model = sizeof(&pgpCardTx[0]);
    pgpCardTx[i].cmd  = IOCTL_Normal_Write;
    pgpCardTx[i].pgpLane = i%2;
    pgpCardTx[i].pgpVc = 3;
    pgpCardTx[i].size = mySize;
    pgpCardTx[i].data = (uint *)malloc(mySize*sizeof(unsigned));
    pgpCardTx[i].data[MagicNumber] = MAGIC;
    pgpCardTx[i].data[QuadNumber] = i;
    printf("PgpCardTx[%u] = ", i);
    printPgpCardTx(&pgpCardTx[i]);
  }

  while (!quit) {

    clock_gettime(CLOCK_REALTIME, &myStart);
    for (unsigned i=0; i<4; i++) {
      pgpCardTx[i].data[FrameCount] = myEventCount;
      ret = write (s,&pgpCardTx[i],sizeof(PgpCardTx));
    }
    clock_gettime(CLOCK_REALTIME, &myEnd);
    interval = timeDiff(&myEnd, &myStart);
    sleepTime.tv_nsec = (interval < 8320500LL) ? 8320500LL - interval : 0;

    nanosleep(&sleepTime, &remainingTime);

    if (!quit) myEventCount++;
    if (!(myEventCount%1000)) {
      printf("myEventCount(%u) interval(%6.3f ms)\n", myEventCount, interval/(float)1e6);
    }
  };
  // sleep long enough to empty out the pipeline
  sleepTime.tv_sec = 1;
  nanosleep(&sleepTime, &remainingTime);
  printf("\nmyEventCount(%u)\n", myEventCount);
  close(s);
}
