
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
   unsigned      value;
   PgpCardTx     p;

   if ( argc != 3 ) {
      cout << "Usage: setScratch device value" << endl;
      return(0);
   }

   if ( (s = open(argv[1], O_RDWR)) <= 0 ) {
      cout << "setScratch: Error opening file" << endl;
      return(1);
   }
   value = strtoul(argv[2], 0, 0);

   cout << "Writing IOCTL_Write_Scratch" << endl;

   p.model = sizeof(&p);
   p.cmd   = IOCTL_Write_Scratch;
   p.data  = (__u32*)value;
   write(s, &p, sizeof(p));

   close(s);
}
