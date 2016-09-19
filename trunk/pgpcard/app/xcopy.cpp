#include <sys/types.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <unistd.h>
#include <stdio.h>
#include <termios.h>
#include <signal.h>
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
#include <sys/socket.h> /* for socket(), connect(), sendto(), and recvfrom() */
#include <arpa/inet.h>  /* for sockaddr_in and inet_addr() */
#include </usr/include/netinet/in.h>
#include <endian.h>
#include <math.h>
#ifdef _POSIX_MESSAGE_PASSING
#include <mqueue.h>
#endif
using namespace std;

unsigned      mySize = 287129;


enum {MaxBytes=(1<<21), NumberOfEvents=16};
enum RunStates {DoNotRun=0, RunForever=1};

class Event {
  public:
    Event();
    ~Event();
    PgpCardRx prx[4];
};

Event::Event() {
  for (unsigned i=0; i<4; i++) {
    // make the buffers bigger than they need to be
    prx[i].maxSize = MaxBytes/sizeof(unsigned);
    prx[i].data = (uint *)malloc(MaxBytes);
    if (!prx[i].data) printf("Unable to allocate event buffer %d\n", i);
  }
}

Event::~Event() {
  for (unsigned i=0; i<4; i++) {
    if (prx[i].data) free(prx[i].data);
  }
}

void sigfunc(int sig_no) {
  printf("Finished. %d\n", sig_no);
  exit(EXIT_SUCCESS);
}

long long int timeDiff(timespec* end, timespec* start) {
  long long int diff;
  diff =  (end->tv_sec - start->tv_sec) * 1000000000LL;
  diff += end->tv_nsec;
  diff -= start->tv_nsec;
  return diff;
}

int main (int argc, char **argv) {
   Event         events[NumberOfEvents];
  long long int interval;
  timespec      sleepTime, myEnd, myLongTimeBack;
  sleepTime.tv_sec = 0;
  sleepTime.tv_nsec = 0;
  unsigned      count = 0;
  (void)        signal(SIGINT, sigfunc);


  unsigned i = 0;
  unsigned j;
  clock_gettime(CLOCK_REALTIME, &myLongTimeBack);
  while (RunForever) {
    i++;
    i %= NumberOfEvents;
    j = i + 1;
    j %= NumberOfEvents;
    for (unsigned k=0; k<4; k++) memcpy(events[i].prx[k].data, events[j].prx[k].data, mySize*sizeof(unsigned));

    if (!(++count%1000)) {
      clock_gettime(CLOCK_REALTIME, &myEnd);
      interval = timeDiff(&myEnd, &myLongTimeBack);
      // size * bitsPerSize * sizesPerEvent * numberOfEvents * (1 read + 1 write)
      float rate = (mySize*32.0*4.0*1000.0*2.0/interval);
      printf("period(%6.3f msec), rate(%5.3f Gbits/s)\n", (float)interval/(1e9), rate);
      myLongTimeBack.tv_nsec = myEnd.tv_nsec;
      myLongTimeBack.tv_sec = myEnd.tv_sec;
    }
  }
  return 0;
}
