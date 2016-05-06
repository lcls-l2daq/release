
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
#include <string.h>
#include <stdlib.h>

#include "../include/PgpCardMod.h"

using namespace std;

int main (int argc, char **argv) {
  PgpCardTx      pgpCardTx;
  int           fd;
  int           ret = 0;
  unsigned      laneMask = 0;
  bool          enable = true;

      if (argc < 4) {
        printf("Usage: %s device laneMask 1|0\n", argv[0]);
        return(1);
      }

  if ( (fd = open(argv[1], O_RDWR)) <= 0 ) {
    cout << "Error opening " << argv[1] << endl;
    perror(argv[1]);
    return(1);
  }

  laneMask = strtoul(argv[2], NULL, 0);
  if (laneMask > 255) {
    cout << "laneMask mask should only be 8 bits! " << argv[2] << endl;
    return(1);
  }
  enable = strtoul(argv[3], NULL, 0) != 0;

  printf("%s is %s EVR laneMask %u\n", argv[0], enable ? "enabling" : "disabling", laneMask);

  pgpCardTx.model   = (sizeof(&pgpCardTx));
  pgpCardTx.pgpVc   = 0;
  pgpCardTx.pgpLane = 0;
  pgpCardTx.size    = sizeof(pgpCardTx);
  pgpCardTx.data    = (__u32*) laneMask;
  pgpCardTx.cmd     = enable ? IOCTL_Evr_LaneEnable : IOCTL_Evr_LaneDisable;
  ret |= write(fd,&pgpCardTx,sizeof(PgpCardTx));

  close(fd);
  return(ret);
}
