
#include <sys/types.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <stdio.h>
#include <termios.h>
#include <fcntl.h>
#include <sstream>
#include <string>
#include <iomanip>
#include <iostream>
#include <string.h>
#include <stdlib.h>

#include "../include/PgpCardMod.h"

using namespace std;

int main (int argc, char **argv) {
  PgpCardTx pgpCardTx;
  int           fd;
  unsigned      l = 0;
  char          c;
  char          name[144] = "";
  bool          nameGiven = false;

  while( ( c = getopt( argc, argv, "d:l:h" ) ) != EOF ) {
    switch(c) {
      case 'd':
        strcpy(name, optarg);
        nameGiven = true;
        break;
      case 'l':
        l = strtol(optarg, NULL, 10);
        break;
      case 'h':
        printf("usage: %s -d device -l laneMask\n", argv[0]);
        return 0;
        break;
      default:
        printf("unknown option %c\n", c);
        printf("usage: %s -d device -l laneMask\n", argv[0]);
        return -1;
        break;
    }
  }

  if (nameGiven) {
    if ( (fd = open(name, O_RDWR)) <= 0 ) {
      cout << "Error opening " << argv[1] << endl;
      return(1);
    }
  } else {
    cout << "device must be given " << endl;
    printf("usage: %s -d device -l laneMask \n", argv[0]);
    return(1);
  }

  PgpCardTx* p = &pgpCardTx;
  p->model = sizeof(p);
  p->cmd   = IOCTL_ClearFrameCounter;
  p->data  = (__u32*) (l&0xff);
  write(fd, p, sizeof(*p));

  cout << endl;
  cout << "Cleared frame counter for lanes 0x" << hex << l << endl << endl;

  close(fd);
}
