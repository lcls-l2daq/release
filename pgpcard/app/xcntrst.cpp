
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
   int  s;
   PgpCardTx     p;

   if ( argc != 2 ) {
      cout << "Usage: xcntrst device" << endl;
      return(0);
   }

   if ( (s = open(argv[1], O_RDWR)) <= 0 ) {
      cout << "\tError opening " << argv[1] << endl;
      return(1);
   }

   p.model = sizeof(&p);
   p.cmd   = IOCTL_Count_Reset;
   p.data  = 0;
   write(s, &p, sizeof(p));
   close(s);
}
