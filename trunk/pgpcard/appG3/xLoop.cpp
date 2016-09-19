
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
#include <string.h>
#include <stdlib.h>

#include "../include/PgpCardMod.h"

#define DEVNAME "/dev/PgpCardG3_0"

using namespace std;

int main (int argc, char **argv) {
   int  s;
   uint i;
   PgpCardTx pgpCardTx;
   
  if (argc < 3) { 
    printf("arc is %d, Usage: %s %s {set|clear}\n", argc, argv[0], argc==2 ? argv[1] : "device");
    return(1);
  } 

  PgpCardTx* p = &pgpCardTx;
  p->model = sizeof(p);
  if ( (s = open(argv[1], O_RDWR)) <= 0 ) {
     cout << "Error opening " << argv[1] << endl;
     return(1);
  }      
  for(i=0;i<8;i++){
    if( strcmp(argv[2],"set") == 0 ) {
       p->cmd   = IOCTL_Set_Loop;
    } else if( strcmp(argv[2],"clear") == 0 ) { 
       p->cmd   = IOCTL_Clr_Loop;
    } else {
       cout << "Usage: xloop device {set|clear}" << endl;
       return(0);      
    }
    p->data  = (__u32*)i;
    write(s, p, sizeof(PgpCardTx));
  }
  close(s);   
}
