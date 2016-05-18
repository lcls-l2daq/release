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
#include "../include/PgpIoctl.h"
using namespace std;

#define DEVNAME "/dev/pgpcard"

int main (int argc, char **argv) {
   int           s;
   uint          x;
   int           ret;
   time_t        t;
   int           count;
   bool          error;
   fd_set        fds;
   int           iLane;
   int           iVc;
   uint          txSize;
   uint          txLane;
   uint          txVc;
   uint          *txData;
   uint          *rxData;
   uint          maxSize;
   uint          rxLane;
   uint          rxVc;
   uint          rxEofe;
   uint          rxFifoErr;
   uint          rxLengthErr;
   struct timeval timeout;

   if (argc != 5) {
      cout << "Usage: xcomp lane vc size count" << endl;
      return(1);
   }

   // Get args
   iLane    = atoi(argv[1]);
   iVc      = atoi(argv[2]);
   txSize   = atoi(argv[3]);
   count    = atoi(argv[4]);

   // Check ranges
   if (txSize == 0 || iVc > 3 || iLane > 3 ) {
      cout << "Invalid size, lane or vc value" << endl;
      return(1);
   }

   if ( iLane >= 0 ) txLane = iLane; else txLane = 0;
   if ( iVc   >= 0 ) txVc   = iVc; else txVc = 0;

   //s = open(DEVNAME, O_RDWR | O_NONBLOCK);
   s = open(DEVNAME, O_RDWR );

   time(&t);
   srandom(t); 

   error = false;
   do {

      txData = (uint *)malloc(sizeof(uint)*txSize);

      // Allocate a read buffer
      maxSize = 1024*1024*2;
      rxData = (uint *)malloc(sizeof(uint)*maxSize);

      // Setup fds for select call
      FD_ZERO(&fds);
      FD_SET(s,&fds);

      // Setup select timeout for 1 second
      timeout.tv_sec=2;
      timeout.tv_usec=0;

      // DMA Write
      for (x=0; x<txSize; x++) txData[x] = random();
      ret = pgpcard_send(s,txData,txSize,txLane,txVc);

      if ( (count-1) % 105 == 0  || count == 1 || error ) {
         cout << "Write Ret=" << dec << ret;
         cout << ", Size=" << dec << txSize;
         cout << ", Lane=" << dec << txLane;
         cout << ", Vc=" << dec << txVc;
         cout << ", Count=" << dec << count;
         cout << endl;
      }

      // Wait for Socket data ready
      ret = select(s+1,&fds,NULL,NULL,&timeout);
      if ( ret <= 0 ) cout << "Read timeout" << endl;

      // DMA Read
      do {
         ret = pgpcard_recv(s,rxData,maxSize,&rxLane,&rxVc,&rxEofe,&rxFifoErr,&rxLengthErr);
         if ( ret <= 0 ) cout << "ret=0" << endl;
      } while ( ret <= 0 );

      if ( ret > 0 ) {
         if ( ret != txSize ) {
	    cout << "Receive Data Size Error." << endl;
	    error = true;
         }
         for (x=0; x<txSize; x++) {
            if ( rxData[x] != txData[x] ) {
               cout << "Receive Data Compare Error. X=" << x << endl;
               error = true;
               break;
            }
         }
      }

      if ( rxEofe != 0 || rxFifoErr != 0 || rxLengthErr != 0 )  error = true;

      if ( (count-1) % 105 == 0  || count == 1 || error ) {
         cout << "Read Ret=" << dec << ret;
         cout << ", Lane=" << dec << rxLane;
         cout << ", Vc=" << dec << rxVc;
         cout << ", Eofe=" << dec << rxEofe;
         cout << ", FifoErr=" << dec << rxFifoErr;
         cout << ", LengthErr=" << dec << rxLengthErr;
         cout << ", Error=" << dec << error;
         cout << ", Count=" << dec << count;
         cout << endl;
        }
      
      free(rxData);
      free(txData);

      if ( iLane == -1 && ( iVc >= 0 || txVc == 3 ) ) {
         if ( txLane == 3 ) txLane = 0;
         else txLane++;
      }

      if ( iVc == -1 ) {
         if ( txVc == 3 ) txVc = 0;
         else txVc++;
      }

   } while ( --count > 0 && !error);

   close(s);
   return(0);
}
