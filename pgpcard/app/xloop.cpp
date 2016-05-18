
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
   int  fd;
   uint port;
   uint loop;

   if ( argc != 4 ) {
      cout << "Usage: xloop port 1/0 deviceName" << endl;
      return(0);
   }
   port = atoi(argv[1]);
   loop = atoi(argv[2]);

   if ( (fd = open(argv[3], O_RDWR)) <= 0 ) {
      cout << "Error opening file" << endl;
      return(1);
   }

   PgpCardTx  t;  
   t.model = sizeof(PgpCardTx*);
   t.data  = (__u32*) port;
   if ( loop == 0 ) { t.cmd   = IOCTL_Clr_Loop;
   } else { t.cmd   = IOCTL_Set_Loop; }
   write(fd, &t, sizeof(PgpCardTx));

   close(fd);
}
