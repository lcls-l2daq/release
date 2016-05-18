
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

#include "../include/PgpCardMod.h"

#define DEVNAME "/dev/pgpcardG3_0_0"

using namespace std;

int main (int argc, char **argv) {
   int       fd;
   __u32     x;
   PgpCardTx  t;
   int      ret = 0;

   if ( (fd = open(DEVNAME, O_RDWR)) <= 0 ) {
      cout << "Error opening file " << fd <<  endl;
      return(1);
   }

   t.model = sizeof(PgpCardTx*);
   t.pgpVc   = 0;
   t.pgpLane = 0;

   for (x=0; x<8; x++) {
      cout << "PGP: TX Reset " << dec << x << endl;
      t.cmd   = IOCTL_Set_Tx_Reset;;
      t.data  = (__u32*) x;
      ret |= write(fd, &t, sizeof(&t));
      t.cmd   = IOCTL_Clr_Tx_Reset;
      t.data  = (__u32*) x;
      ret |= write(fd, &t, sizeof(&t));
   }

   for (x=0; x<8; x++) {
      cout << "PGP: RX Reset " << dec << x << endl;
      t.cmd   = IOCTL_Set_Rx_Reset;;
      t.data  = (__u32*) x;
      ret |= write(fd, &t, sizeof(&t));
      t.cmd   = IOCTL_Clr_Rx_Reset;
      t.data  = (__u32*) x;
      ret |= write(fd, &t, sizeof(&t));
   }
   
   cout << "EVR: Reset " << endl;
   
   t.cmd   = IOCTL_Evr_Set_PLL_RST;;
   t.data  = (__u32*) 0;
   ret |= write(fd, &t, sizeof(&t));
   t.cmd   = IOCTL_Evr_Clr_PLL_RST;
   ret |= write(fd, &t, sizeof(&t));
   
   t.cmd   = IOCTL_Evr_Set_Reset;;
   ret |= write(fd, &t, sizeof(&t));
   t.cmd   = IOCTL_Evr_Clr_Reset;
   ret |= write(fd, &t, sizeof(&t));
   
   cout << "Resetting status counters" << endl;
   t.cmd   = IOCTL_Count_Reset;;
   ret |= write(fd, &t, sizeof(&t));

   close(fd);
   return ret;
}
