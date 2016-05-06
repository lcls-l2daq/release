
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

      if (argc < 2) {
        printf("Usage: %s device\n", argv[0]);
        return(1);
      }

  if ( (fd = open(argv[1], O_RDWR)) <= 0 ) {
    cout << "Error opening " << argv[1] << endl;
    perror(argv[1]);
    return(1);
  }

  printf("%s is enabling the EVR\n", argv[0]);

  pgpCardTx.model   = (sizeof(&pgpCardTx));
  pgpCardTx.pgpVc   = 0;
  pgpCardTx.pgpLane = 0;
  pgpCardTx.size    = sizeof(pgpCardTx);
  pgpCardTx.data    = (__u32*) 0;
  pgpCardTx.cmd     = IOCTL_Evr_Set_PLL_RST;
  ret |= write(fd,&pgpCardTx,sizeof(PgpCardTx));
  pgpCardTx.cmd     = IOCTL_Evr_Clr_PLL_RST;
  ret |= write(fd,&pgpCardTx,sizeof(PgpCardTx));
  pgpCardTx.cmd     = IOCTL_Evr_Set_Reset;
  ret |= write(fd,&pgpCardTx,sizeof(PgpCardTx));
  pgpCardTx.cmd     = IOCTL_Evr_Clr_Reset;
  ret |= write(fd,&pgpCardTx,sizeof(PgpCardTx));
  pgpCardTx.cmd     = IOCTL_Evr_Enable;
  ret |= write(fd,&pgpCardTx,sizeof(PgpCardTx));

  close(fd);
  return(ret);
}
