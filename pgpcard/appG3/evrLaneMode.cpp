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
  unsigned      lane = 0;
  unsigned      mode = 0;

      if (argc < 4) {
        printf("Usage: %s device lane mode\n", argv[0]);
        return(1);
      }

  if ( (fd = open(argv[1], O_RDWR)) <= 0 ) {
    cout << "Error opening " << argv[1] << endl;
    perror(argv[1]);
    return(1);
  }
  lane = strtoul(argv[2], NULL, 0);
  mode = strtoul(argv[3], NULL, 0);
  if ((lane > 7) | (mode > 1)) {
    cout << "Parameter is too large! " << lane <<','<< mode << endl;
    return(1);
  }

  printf("%s is setting EVR lane %u to %swait for fiducials\n", argv[0], lane, mode ? "" : "not ");

  pgpCardTx.model   = (sizeof(&pgpCardTx));
  pgpCardTx.pgpVc   = 0;
  pgpCardTx.pgpLane = 0;
  pgpCardTx.size    = sizeof(pgpCardTx);
  pgpCardTx.data    = (__u32*) (1<<lane);
  pgpCardTx.cmd     = mode ? IOCTL_Evr_LaneModeFiducial : IOCTL_Evr_LaneModeNoFiducial;
  ret |= write(fd,&pgpCardTx,sizeof(PgpCardTx));

  close(fd);
  return(ret);
}
