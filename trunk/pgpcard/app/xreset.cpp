
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
using namespace std;

#define DEVNAME "/dev/pgpcard0"

int main (int argc, char **argv) {
   int       s;
   __u32     x;
   PgpCardTx p;

   if ( (s = open(DEVNAME, O_RDWR)) <= 0 ) {
      cout << "Error opening file" << endl;
      return(1);
   }

   p.model = sizeof(&p);

   for (x=0; x< 4; x++) {
     cout << "Tx Reset " << dec << x << endl;
     p.cmd   = IOCTL_Set_Tx_Reset;
     p.data  = (__u32*)x;
     write(s, &p, sizeof(p));
     p.cmd   = IOCTL_Clr_Tx_Reset;
     p.data  = (__u32*)x;
     write(s, &p, sizeof(p));
   }

   for (x=0; x< 4; x++) {
     cout << "Rx Reset " << dec << x << endl;
     p.cmd   = IOCTL_Set_Rx_Reset;
     p.data  = (__u32*)x;
     write(s, &p, sizeof(p));
     p.cmd   = IOCTL_Clr_Rx_Reset;
     p.data  = (__u32*)x;
     write(s, &p, sizeof(p));
   }

   p.cmd   = IOCTL_Count_Reset;
   write(s, &p, sizeof(p));

   close(s);
}
