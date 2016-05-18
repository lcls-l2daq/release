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
#include <string.h>
#ifdef _POSIX_MESSAGE_PASSING
#include <mqueue.h>
#endif
using namespace std;

#define DEVNAME "/dev/pgpcard_0_15"

unsigned      eofeCount = 0;
unsigned      fifoErrCount = 0;
unsigned      lengthErrCount = 0;
unsigned      transmittedCount = 0;
unsigned      damageCount = 0;
unsigned      ethPacketsTransmitted = 0;

#define PERMS (S_IRUSR|S_IRUSR|S_IRUSR|S_IROTH|S_IROTH|S_IROTH|S_IRGRP|S_IRGRP|S_IRGRP| \
    S_IWUSR|S_IWUSR|S_IWUSR|S_IWOTH|S_IWOTH|S_IWOTH|S_IWGRP|S_IWGRP|S_IWGRP)
#define OFLAGS (O_CREAT|O_RDWR)

enum MSG_TYPE {AreYouThere, YesIAm, EventBufferFull, EventBufferEmpty};
enum {
    ChunkSize = 8192,
    QuadsPerEvent = 4,
    QuadWords = 287129,
    QuadSize = QuadWords * sizeof(unsigned),
    FullChunksPerQuad = QuadSize / ChunkSize,
    SizeOfLastPartial = QuadSize % ChunkSize,
    EventSize         = (QuadSize * QuadsPerEvent),
    IPAddrToSendTo    = (0xefff2811),
    IPPortToSendTo    = 1111,
    SendFlags         = 0
};
enum {MagicNumber=0, NumberOfQuads=4, FrameCount=2, NumberOfEvents=16, MAGIC=0xFEEDFACE,
  MaxBytes=QuadSize*NumberOfQuads + 20};

enum RunStates {DoNotRun=0, RunForever=1};
int           pgpCardDriverFilePointer;

class Event {
  public:
    Event();
    ~Event();
    PgpCardRx prx[4];
};

Event::Event() {
  for (unsigned i=0; i<4; i++) {
    prx[i].model = sizeof(this);
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

class Msg {
  public:
    Msg() {magick = MAGIC;}
    ~Msg() {}
    unsigned magick;
    enum MSG_TYPE type;
    Event* e;
} myOut, myIn;

struct mq_attr mymq_attr;

void sigfunc(int sig_no) {
  printf("\ntransmittedCount(%u), eofe(%u), fifoErr(%u), lengthErr(%u), damageCount(%u)\n",
      transmittedCount, eofeCount, fifoErrCount, lengthErrCount, damageCount);
  printf("ethPacketsTransmitted(%u)\n", ethPacketsTransmitted);
  close(pgpCardDriverFilePointer);
  printf("Unlinking ... \n");
  if (mq_unlink("/MsgToTransmitterQueue") == (mqd_t)-1) perror("mq_unlink To Transmitter");
  if (mq_unlink("/MsgFromTransmitterQueue") == (mqd_t)-1) perror("mq_unlink From Transmitter");
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

void printSock(sockaddr_in* a) {
  char* cp = (char*) a;
  for (unsigned i=0; i<sizeof(sockaddr_in); i++) {
    printf("%2hhx ", cp[i]);
  }
  printf("\n");
}

void printHdr(msghdr* a) {
  char* cp = (char*) a;
  for (unsigned i=0; i<sizeof(msghdr); i++) {
    printf("%2hhx ", cp[i]);
  }
  printf("\n");
}

long long unsigned _badEvents = 0LL;

unsigned IPAddrToSendFrom = 0;

void* txMain(void* p) {
  Msg             _myOut, _myIn;
  unsigned        _priority = 0;
  int             _sock;
  int             parm = ChunkSize*1000;
  sockaddr_in     _socketAddr;
  sockaddr_in     _mySocketAddr;
  in_addr         _myAddress;
  _myAddress.s_addr  = htonl(IPAddrToSendFrom);
  struct msghdr   hdr;
  struct iovec    iov[1];
  hdr.msg_iov   = iov;
  hdr.msg_iovlen       = 1;
  hdr.msg_name         = (caddr_t)&_socketAddr;
  hdr.msg_namelen      = sizeof(sockaddr_in);
  hdr.msg_control      = (caddr_t)0;
  hdr.msg_controllen   = 0;

  struct mq_attr _mymq_attr;
  _mymq_attr.mq_maxmsg = 10L;
  _mymq_attr.mq_msgsize = (long int)sizeof(_myOut);
  _mymq_attr.mq_flags = 0L;

  mqd_t _myInputQueue = mq_open("/MsgToTransmitterQueue", O_RDWR, PERMS, &_mymq_attr);
  if (_myInputQueue < 0) perror("mq_open transmitter input");
  mqd_t _myOutputQueue = mq_open("/MsgFromTransmitterQueue", O_RDWR, PERMS, &_mymq_attr);
  if (_myOutputQueue < 0) perror("mq_open transmitter output");
  printf("_myInputQueue 1 %d\n", _myInputQueue);

  if ((_sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {
    perror("transmitter socket() failed");
    pthread_exit(NULL);
  }

  if(setsockopt(_sock, SOL_SOCKET, SO_SNDBUF, (char*)&parm, sizeof(parm)) < 0) {
    perror("setsockopt(SO_SNDBUF) failed transmitter thread");
    pthread_exit(NULL);
  }

  if (setsockopt(_sock, IPPROTO_IP, IP_MULTICAST_IF, &_myAddress.s_addr, sizeof(unsigned)) < 0) {
    char str[120];
    sprintf(str, "setsockopt(IP_MULTICAST_IF) failed transmitter thread address 0x%x ", _myAddress.s_addr);
    perror(str);
    pthread_exit(NULL);
  }

  memset(&_mySocketAddr, 0, sizeof(_mySocketAddr));       // Zero out structure
  _mySocketAddr.sin_family = AF_INET;                     // Internet address family
  _mySocketAddr.sin_addr.s_addr = htonl(IPAddrToSendFrom);// Incoming interface
  _mySocketAddr.sin_port = 0;         //  port

  memset(&_socketAddr, 0, sizeof(_socketAddr));         // Zero out structure
  _socketAddr.sin_family = AF_INET;                     // Internet address family
  _socketAddr.sin_addr.s_addr = htonl(IPAddrToSendTo);  // Incoming interface
  _socketAddr.sin_port = htons(IPPortToSendTo);         //  port

  if (bind(_sock, (struct sockaddr *) &_mySocketAddr, sizeof(_mySocketAddr)) < 0) {
    perror("bind() failed transmitter thread");
    pthread_exit(NULL);
  }

  if (mq_receive(_myInputQueue, (char*)&_myIn, sizeof(_myIn), &_priority) < 0) perror("transmitter mq_receive checkin");
  if (_myIn.type == AreYouThere) {
    _myOut.type = YesIAm;
    if (mq_send(_myOutputQueue, (const char *)&_myOut, sizeof(_myOut), 0)) perror("transmitter mq_send checkin");
  }

  printf("FullChunksPerQuad(%d), SizeOfLastPartial(%d), packetsPerEvent(%d)\n",
      FullChunksPerQuad, SizeOfLastPartial,
      (FullChunksPerQuad*4) + (SizeOfLastPartial ? 4 : 0));

  while (RunForever) {
    if (mq_receive(_myInputQueue, (char*)&_myIn, sizeof(_myIn), &_priority) < 0) {
      perror("transmitter mq_receive data");
      _badEvents++;
      if (_badEvents>10) while (RunForever) {};
    } else
    {
      for (unsigned i=0; i < QuadsPerEvent; i++) {
        char* bp = (char*) _myIn.e->prx[i].data;
        iov[0].iov_len = ChunkSize;
        for (unsigned j=0; j < FullChunksPerQuad; j++) {
          iov[0].iov_base = (caddr_t)bp;
          if (sendmsg(_sock, &hdr, SendFlags) < 0) {
            perror("Transmit Thread full sendmsg _sock");
            printf("_socketAddr:   "); printSock(&_socketAddr);
            printf("hdr: "); printHdr(&hdr);
            pthread_exit(NULL);
          }
          bp += ChunkSize;
          ethPacketsTransmitted += 1;
        }
        if (SizeOfLastPartial) {
          iov[0].iov_len = SizeOfLastPartial;
          iov[0].iov_base = (caddr_t)bp;
          if (sendmsg(_sock, &hdr, SendFlags) < 0) {
            perror("Transmit Thread last sendmsg _sock");
            pthread_exit(NULL);
          }
          ethPacketsTransmitted += 1;
        }
      }
    }
  }
  pthread_exit(NULL);
  return NULL;
}

int main (int argc, char **argv) {
  pthread_t     txThread;
  unsigned      priority = 0;
  Event         events[NumberOfEvents];
  unsigned      eventIndex = 0;
  unsigned      myCount = 0;
  long long int interval;
  timespec      myStart, myEnd;
  (void)        signal(SIGINT, sigfunc);
  if (argc != 2) {
     printf("usage: %s <addressToSendFrom>\n", argv[0]);
     return(1);
  }

  printf("Memory Model is: %u\n", events[0].prx[0].model);

  struct mq_attr mymq_attr;
  mymq_attr.mq_maxmsg = 10L;
  mymq_attr.mq_msgsize = (long int)sizeof(myOut);
  mymq_attr.mq_flags = 0L;

  mqd_t myOutputQueue = mq_open("/MsgToTransmitterQueue", O_CREAT|O_RDWR, PERMS, &mymq_attr);
  if (myOutputQueue < 0) perror("mq_open output to transmitter");
  else { printf("mq_open output to transmitter SUCCEEDED %d\n", myOutputQueue); }
  mqd_t myInputQueue = mq_open("/MsgFromTransmitterQueue", O_CREAT|O_RDWR, PERMS, &mymq_attr);
  if (myInputQueue < 0) perror("mq_open input from transmitter");
  else { printf("mq_open input from transmitter SUCCEEDED %d\n", myInputQueue); }

  if ( int r = pthread_create(&txThread,NULL,txMain,NULL) ) {
     printf("Error creating transmitter thread %d\n", r);
     return(2);
  } else {
    printf("Created transmitter thread txThread(0x%x) (%d)\n", (unsigned)txThread, (int)txThread);
  }

  myOut.type = AreYouThere;
  if (mq_send(myOutputQueue, (const char *)&myOut, sizeof(myOut), 0)) perror("mq_send checkin");
  else if (mq_receive(myInputQueue, (char*)&myIn, sizeof(myIn), &priority) < 0) perror("mq_receive checkin");
  else {
    if (myIn.type == YesIAm) {
      printf("Connected with transmitter.\n");
      myOut.type = EventBufferFull;
      clock_gettime(CLOCK_REALTIME, &myStart);

      while (RunForever) {
        transmittedCount++;
        myOut.e = &events[eventIndex];
        if (mq_send(myOutputQueue, (const char *)&myOut, sizeof(myOut), 0)) perror("mq_send event");
        eventIndex++;
        eventIndex %= NumberOfEvents;
        if (!(transmittedCount%1000)) {
          if (transmittedCount) {
            clock_gettime(CLOCK_REALTIME, &myEnd);
            interval = timeDiff(&myEnd, &myStart);
            float rate = (QuadSize*8*4000.0/interval);
            printf("period(%6.3f msec), rate(%5.3f Gbit/s), ", interval/(1e6*1000.0), rate);
          }
          printf("transmittedCount(%u)\n", transmittedCount);
          myStart.tv_nsec = myEnd.tv_nsec;
          myStart.tv_sec  = myEnd.tv_sec;
        }
        ++myCount;
      }
    }
  }
  pthread_join(txThread, NULL);
  sigfunc(0);
  return 0;
}
