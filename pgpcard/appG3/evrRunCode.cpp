
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
  unsigned      e = 0;
  char          c;
  char          name[144] = "";
  bool          nameGiven = false;

  while( ( c = getopt( argc, argv, "d:l:e:h" ) ) != EOF ) {
    switch(c) {
      case 'd':
        strcpy(name, optarg);
        nameGiven = true;
        break;
      case 'e':
        e = strtoul(optarg, NULL, 0);
        break;
      case 'l':
        l = (strtol(optarg, NULL, 10)&7);
        break;
      case 'h':
        printf("usage: %s -d device -l lane -e eventCode\n", argv[0]);
        return 0;
        break;
      default:
        printf("unknown option %c\n", c);
        printf("usage: %s -d device -l lane -e eventCode\n", argv[0]);
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
    printf("usage: %s -d device -l lane -e eventCode\n", argv[0]);
    return(1);
  }

  PgpCardTx* p = &pgpCardTx;
  unsigned long stuff = ((l << 28) | (e&0xff));
  p->model = sizeof(p);
  p->cmd   = IOCTL_Evr_RunCode;
  p->data  = (__u32*) stuff;
  write(fd, p, sizeof(*p));

  cout << endl;
  cout << "Wrote Evr run code " << e << " to lane " << l << endl << endl;

  close(fd);
}
