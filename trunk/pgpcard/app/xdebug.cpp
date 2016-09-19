
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
   int           s;
   unsigned      debug;
   PgpCardTx     p;

   if ( argc != 3 ) {
      cout << "Usage: xdebug device level" << endl;
      return(0);
   }

   if ( (s = open(argv[1], O_RDWR)) <= 0 ) {
      cout << "Error opening file" << endl;
      return(1);
   }
   debug = strtoul(argv[2], 0, 0);

   cout << "Writing IOCTL_Set_Debug" << endl;

   p.model = sizeof(&p);
   p.cmd   = IOCTL_Set_Debug;
   p.data  = (__u32*)debug;
   write(s, &p, sizeof(p));

   close(s);
}
