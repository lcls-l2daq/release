#include <sys/types.h>
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
   int           x;
   int           ret;
   uint          maxSize;
   uint          *data;
   PgpCardRx pgpCardRx;

   if (argc<2) {
      printf("usage: %s devName\n", argv[0]);
      exit(0);
   }

   fd = open(argv[1], O_RDWR);
   if (fd < 0) {
      printf("Failed to open %s\n", argv[1]);
      exit(1);
   }
   // Allocate a buffer
   maxSize = 1024*1024*2;
   data = (uint *)malloc(sizeof(uint)*maxSize);

   pgpCardRx.maxSize = maxSize;
   pgpCardRx.data    = (__u32*)data;
   pgpCardRx.model   = sizeof(data);


   // DMA Read
   do {
      ret = read(fd,&pgpCardRx,sizeof(PgpCardRx));

      if ( ret != 0 ) {

         cout << "Ret=" << dec << ret;
         cout << ", Lane=" << dec << pgpCardRx.pgpLane;
         cout << ", Vc=" << dec << pgpCardRx.pgpVc;
         cout << ", Eofe=" << dec << pgpCardRx.eofe;
         cout << ", FifoErr=" << dec << pgpCardRx.fifoErr;
         cout << ", LengthErr=" << dec << pgpCardRx.lengthErr;
         cout << endl << "   ";

         //for (x=0; x<ret; x++) {
            //cout << " 0x" << setw(8) << setfill('0') << hex << data[x];
            //if ( ((x+1)%10) == 0 ) cout << endl << "   ";
         //}
         //cout << endl;
      }
   } while ( ret > 0 );
   
   free(data);

   close(fd);
}
