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
#include "../include/PgpCardMod.h"
using namespace std;

int main (int argc, char **argv) {
   int           fd;
   uint          x;
   int           ret;
   time_t        t;
   uint          lane;
   uint          vc;
   uint          size;
   uint          *data;
   PgpCardTx     pgpCardTx;


   if (argc != 5) {
      printf("Usage: %s devName lane vc size\n", argv[0]);
      return(1);
   }

   // Get args
   lane  = atoi(argv[2]);
   vc    = atoi(argv[3]);
   size  = atoi(argv[4]);

   // Check ranges
   if ( size == 0 || lane > 3 || vc > 3 ) {
      cout << "Invalid size, lane or vc value" << endl;
      return(1);
   }

   fd = open(argv[1], O_RDWR);
   if (fd<0) {
     printf("Could not open %s\n", argv[1]);
     return(1);
   }

   pgpCardTx.model   = (sizeof(data));
   pgpCardTx.cmd     = IOCTL_Normal_Write;
   pgpCardTx.pgpVc   = vc; 
   pgpCardTx.pgpLane = lane;
   pgpCardTx.size    = size;
   pgpCardTx.data    = (__u32*)data;

   time(&t);
   srandom(t); 

   data = (uint *)malloc(sizeof(uint)*size);

   // DMA Write
   cout << "Sending:" << endl << "   ";
   for (x=0; x<size; x++) {
      data[x] = random();
      cout << " 0x" << setw(8) << setfill('0') << hex << data[x];
      if ( ((x+1)%10) == 0 ) cout << endl << "   ";
   }
   cout << endl;
   ret = write(fd,&pgpCardTx,sizeof(PgpCardTx));
   cout << "Ret=" << dec << ret << endl;
  
   free(data);

   close(fd);
   return(0);
}
