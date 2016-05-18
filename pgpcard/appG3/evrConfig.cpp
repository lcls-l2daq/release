
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
  uint          lane, rmode;
  uint          rdelay, adelay;
  uint          rcode, acode;

      if (argc < 8) {
        printf("Usage: %s device lane runCode acceptCode runDelay acceptDelay runMode\n", argv[0]);
        return(0);
      }

  // Get args
  lane   = atoi(argv[2]);
  rcode  = atoi(argv[3]);
  acode  = atoi(argv[4]);
  rdelay  = atoi(argv[5]);
  adelay  = atoi(argv[6]);
  rmode   = atoi(argv[7]);

  // Check ranges
  if ( lane > 7 || rcode > 255 || acode > 255) {
    printf("%s: Invalid lane or code value : %u or %u,%u\n", argv[0], lane, rcode, acode);
    return(1);
  }

  if ( (fd = open(argv[1], O_RDWR)) <= 0 ) {
    cout << "Error opening " << argv[1] << endl;
    perror(argv[1]);
    return(1);
  }

  printf("%s is writing run code %u, accept code %u, run delay of %u, an accept delay of %u, and run mode of %u\n", argv[0], rcode, acode, rdelay, adelay, rmode);

  pgpCardTx.model   = (sizeof(&pgpCardTx));
  pgpCardTx.pgpVc   = 0;
  pgpCardTx.pgpLane = lane;
  pgpCardTx.size    = sizeof(pgpCardTx);
  pgpCardTx.cmd     = IOCTL_Evr_RunCode;
  pgpCardTx.data    = (__u32*) ((lane << 28) | (rcode & 0xfffffff));
  ret |= write(fd,&pgpCardTx,sizeof(PgpCardTx));
  pgpCardTx.cmd     = rmode ? IOCTL_Evr_LaneModeFiducial : IOCTL_Evr_LaneModeNoFiducial;
  pgpCardTx.data    = (__u32*) (1 << lane);
  ret |= write(fd,&pgpCardTx,sizeof(PgpCardTx));
  pgpCardTx.cmd     = IOCTL_Evr_AcceptCode;
  pgpCardTx.data    = (__u32*) ((lane << 28) | (acode & 0xfffffff));
  ret |= write(fd,&pgpCardTx,sizeof(PgpCardTx));
  pgpCardTx.cmd     = IOCTL_Evr_RunDelay;
  pgpCardTx.data    = (__u32*) ((lane << 28) | (rdelay & 0xfffffff));
  ret |= write(fd,&pgpCardTx,sizeof(PgpCardTx));
  pgpCardTx.cmd     = IOCTL_Evr_AcceptDelay;
  pgpCardTx.data    = (__u32*) ((lane << 28) | (adelay & 0xfffffff));
  ret |= write(fd,&pgpCardTx,sizeof(PgpCardTx));

  close(fd);
  return(ret);
}
