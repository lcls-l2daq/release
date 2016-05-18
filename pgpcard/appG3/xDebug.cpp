
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

int main (int argc, char **argv) {
   int       fd;
   __u32     level;


   PgpCardTx  t;

   if (argc < 3) {
     printf("Usage: %s device debugLevel\n", argv[0]);
     return 0;
   }

   level = atoi(argv[2]);

   t.model = sizeof(PgpCardTx*);
   t.cmd   = IOCTL_Set_Debug;
   t.data  = (__u32*) level;

   if ( (fd = open(argv[1], O_RDWR)) <= 0 ) {
      cout << "Error opening file" << endl;
      return(1);
   }

   write(fd, &t, sizeof(PgpCardTx));

   printf("wrote debug level 0x%x to %s\n", level, argv[1]);

   close(fd);
}
