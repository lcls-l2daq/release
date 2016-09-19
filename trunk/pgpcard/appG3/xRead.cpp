
#include <sys/types.h>
#include <unistd.h>
#include <stdio.h>
#include <termios.h>
#include <fcntl.h>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <new>

FILE*               writeFile           = 0;
bool                writing             = false;

void sigHandler( int signal ) {
  psignal( signal, "Signal received by pgpWidget");
  if (writing) fclose(writeFile);
  printf("Signal handler pulling the plug\n");
  ::exit(signal);
}


#include "../include/PgpCardMod.h"

using namespace std;

void printUsage(char* name) {
  printf( "Usage: %s [-h]  -P <deviceName> [-c count] [-o maxPrint] [-D <debug>]\n"
      "    -h      Show usage\n"
      "    -P      Set pgpcard device name  (REQUIRED)\n"
      "    -c      number of times to read\n"
      "    -l      lane\n"
      "    -r      Loop reading data until interrupted or count exceeded\n"
      "    -o      Print out up to maxPrint words when reading data\n"
//      "    -s      Save to file when reading data\n"
      "    -D      Set debug value           [Default: 0]\n"
      "                bit 00          print out progress\n",
      name
  );
}
enum Commands{none, writeCommand,readCommand,readAsyncCommand,dumpCommand,testCommand,loopWriteCommand,numberOfCommands};

int main (int argc, char **argv) {
  PgpCardRx     pgpCardRx;
  PgpCardTx     pgpCardTx;
  int           x;
  int           ret, fd;
  unsigned           count = 0;
  int           numb;
  bool          print = false;
  uint          maxSize;
  uint          *data;
  char          err[128];
  char          pgpcard[128]              = "";
  unsigned            d                   = 0;
  unsigned            command             = none;
  unsigned            addr                = 0;
  unsigned            maxPrint            = 1024;
  bool                cardGiven           = false;
  unsigned            debug               = 0;
  ::signal( SIGINT, sigHandler );
  char                runTimeConfigname[256] = {""};
  char                writeFileName[256] = {""};

  char*               endptr;
  extern char*        optarg;
  int c;
  while( ( c = getopt( argc, argv, "hP:D:r:c:l:o:" ) ) != EOF ) {
    switch(c) {
      case 'P':
        strcpy(pgpcard, optarg);
        cardGiven = true;
        break;
      case 'D':
        debug = strtoul(optarg, NULL, 0);
        if (debug & 1) print = true;
        break;
      case 'r':
        command = readCommand;
        d = strtoul(optarg  ,&endptr,0);
        addr = strtoul(endptr+1,&endptr,0);
        if (debug & 1) printf("\t read %u,0x%x\n", d, addr);
        break;
      case 'c':
        numb = strtoul(optarg  ,NULL,0);
        break;
      case 'o':
        maxPrint = strtoul(optarg, NULL, 0);
        print = true;
        break;
      case 'h':
        printUsage(argv[0]);
        return 0;
        break;
      default:
        printf("Error: Option could not be parsed, or is not supported yet!\n");
        printUsage(argv[0]);
        return 0;
        break;
    }
  }

  if (!cardGiven) {
    printf("PGP card must be specified !!\n");
    printUsage(argv[0]);
    return 1;
  }
  fd = open( pgpcard,  O_RDWR );
  if (fd < 0) {
    sprintf(err, "%s opening %s failed", argv[0], pgpcard);
    perror(err);
    return 1;
  }

  pgpCardTx.cmd = IOCTL_Normal_Write;
  pgpCardTx.model = sizeof(&pgpCardTx);
  pgpCardTx.size = sizeof(PgpCardTx);

  // Allocate a buffer
  maxSize = 1024*1024*2;
  data = (uint *)malloc(sizeof(uint)*maxSize);

  pgpCardRx.maxSize = maxSize;
  pgpCardRx.data    = (__u32*)data;
  pgpCardRx.model   = sizeof(data);

  // only one command so far ...

  // DMA Read
  do {
    ret = read(fd,&pgpCardRx,sizeof(PgpCardRx));

    if ( ret != 0 ) {
      if (print) {

        cout << "Ret=" << dec << ret;
        cout << ", pgpLane=" << dec << pgpCardRx.pgpLane;
        cout << ", pgpVc=" << dec << pgpCardRx.pgpVc;
        cout << ", EOFE=" << dec << pgpCardRx.eofe;
        cout << ", FifoErr=" << dec << pgpCardRx.fifoErr;
        cout << ", LengthErr=" << dec << pgpCardRx.lengthErr;
        cout << endl << "   ";

        for (x=0; x<ret && x<maxPrint; x++) {
          cout << " 0x" << setw(8) << setfill('0') << hex << data[x];
          if ( ((x+1)%10) == 0 ) cout << endl << "   ";
        }
        cout << endl;
      } else {
        if (count%1000 == 0) {
          printf("%s has read %u\n", argv[0], count);
        }
      }
    }
  } while ( ret > 0 && count++ < numb );
  if (ret < 0) {
    sprintf(err, "%s reading %s failed ", argv[0], pgpcard);
    perror(err);
    return 1;
  }
  free(data);
  close(fd);
  return 0;
}
