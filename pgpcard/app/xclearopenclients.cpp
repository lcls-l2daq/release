
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
   int           s, ret;
   PgpCardTx  t;

   if ( argc != 2) {
      cout << "Usage: " << argv[0] <<" device" << endl;
      return(0);
   }

   if ( (s = open(argv[1], O_RDWR)) <= 0 ) {
      cout << "Error opening file" << endl;
      return(1);
   }

   t.model = sizeof(int*);
   t.cmd   = IOCTL_Clear_Open_Clients;
   t.data  = 0;
   ret = (write(s, &t, sizeof(PgpCardTx)));

   cout << "Opening and closing " << argv[1] << endl;
   close(s);
   return ret;
}
