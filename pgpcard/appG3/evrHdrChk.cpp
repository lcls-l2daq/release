
#include <sys/types.h>
#include <linux/types.h>
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

#include "../include/PgpCardMod.h"

using namespace std;

void sigHandler( int signal ) {
  psignal( signal, "Signal received by pgpWidget");
  printf("Signal handler pulling the plug\n");
  ::exit(signal);
}

void printUsage(char* name) {
  printf( "Usage: %s [-h]  -P <deviceName> -l <lane> -v <vc> -e <enable>\n"
      "    -h      Show usage\n"
      "    -P      Set pgpcard device name  (REQUIRED)\n"
      "    -l      lane\n"
      "    -v      vc\n"
      "    -e      enable\n",
      name
  );
}

int main (int argc, char **argv) {
  PgpCardTx      pgpCardTx;
  unsigned         lane = 0;
  unsigned         vc   = 0;
  int              fd;
  int              ret = 0;
  char             pgpcard[128] = "";
  int              c;
  unsigned         enable = 0;
  bool             cardGiven  = false;

  ::signal( SIGINT, sigHandler );
  extern char*        optarg;

  extern char*        optarg;
  while( ( c = getopt( argc, argv, "hP:l:v:e:" ) ) != EOF ) {
    switch(c) {
      case 'P':
        strcpy(pgpcard, optarg);
        cardGiven = true;
        break;
      case 'l':
        lane = strtoul(optarg  ,NULL,0);
        lane &= 0xf;
        break;
      case 'v':
        vc = strtoul(optarg  ,NULL,0);
        vc &= 3;
        break;
      case 'e':
        enable = strtoul(optarg ,NULL,0);
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
    printUsage(argv[0]);
    return(1);
  }

  if ( (fd = open(pgpcard, O_RDWR)) <= 0 ) {
    cout << "Error opening " << pgpcard << endl;
    perror(argv[1]);
    return(1);
  }

  printf("%s is %s the EVR header check for lane %u vc %u\n", argv[0], enable ? "enabling" : "disabling", lane, vc);

  pgpCardTx.model   = (sizeof(&pgpCardTx));
  pgpCardTx.pgpVc   = 0;
  pgpCardTx.pgpLane = 0;
  pgpCardTx.size    = sizeof(pgpCardTx);

  unsigned stuff = 0;
  stuff = (lane<<28) | (vc<<24) | (enable ? 1 : 0);

  pgpCardTx.data   = (__u32*) stuff;
  pgpCardTx.cmd     = IOCTL_Evr_En_Hdr_Check;
  ret |= write(fd,&pgpCardTx,sizeof(PgpCardTx));

  close(fd);
  return(ret);
}
