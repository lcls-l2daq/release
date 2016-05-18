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

#define DEVNAME "/dev/pgpcard"

int main (int argc, char **argv) {
   int           s;
   int           ret;
   time_t        t;
   PgpCardTx     pgpCardTx;
   PgpCardRx     pgpCardRx; 
   bool          error;
   int           count;
   PgpCardStatus status;
   // the buffer is a PgpCardTx on the way in and a PgpCardStatus on the way out
   __u8*         c = (__u8*) &status;  // this adheres to strict aliasing rules
   PgpCardTx*    p = (PgpCardTx*) c;

   s = open(DEVNAME, O_RDWR);

   time(&t);
   srandom(t); 

   // Allocate a read buffer
   pgpCardRx.maxSize = 1024*1024*2;
   pgpCardRx.data = (uint *)malloc(sizeof(uint)*pgpCardRx.maxSize);
   pgpCardRx.model = sizeof(p);
   pgpCardTx.data = pgpCardRx.data;
   pgpCardTx.cmd  = IOCTL_Normal_Write;
   pgpCardTx.model = sizeof(p);

   error = false;
   count = 0;
   while (1) {

      // DMA Read
      do {
         ret = read(s,&pgpCardRx,sizeof(PgpCardRx));
         usleep(1);
      } while ( ret == 0 );

      if ( ret > 0 ) {
         count ++;
         if ( pgpCardRx.eofe != 0 || pgpCardRx.fifoErr != 0 || pgpCardRx.lengthErr != 0 )  error = true;
         if ( (count % 100) == 0 || error ) {

            memset(&status,0,sizeof(PgpCardStatus));
	    p->model = sizeof(p);
	    p->cmd   = IOCTL_Read_Status;
	    p->data  = (__u32*)&status;
            write(s, p, sizeof(status));

            cout << endl;
            cout << "Read Ret=" << dec << ret;
            cout << ", Size=" << dec << pgpCardRx.rxSize;
            cout << ", Lane=" << dec << pgpCardRx.pgpLane;
            cout << ", Vc=" << dec << pgpCardRx.pgpVc;
            cout << ", Eofe=" << dec << pgpCardRx.eofe;
            cout << ", FifoErr=" << dec << pgpCardRx.fifoErr;
            cout << ", LengthErr=" << dec << pgpCardRx.lengthErr;
            cout << ", Error=" << dec << error;
            cout << ", Count=" << dec << count;
            cout << ", CellErr: " << hex << setw(1) << setfill('0') << status.Pgp1CellErrCnt;
            cout <<                  hex << setw(1) << setfill('0') << status.Pgp0CellErrCnt;
            cout << ", LinkDown: " << hex << setw(1) << setfill('0') << status.Pgp1LinkDownCnt;
            cout <<                   hex << setw(1) << setfill('0') << status.Pgp0LinkDownCnt;
            cout << ", LinkErr: " << hex << setw(1) << setfill('0') << status.Pgp1LinkErrCnt;
            cout <<                  hex << setw(1) << setfill('0') << status.Pgp0LinkErrCnt;
            cout << ", FifoErr: " << hex << setw(1) << setfill('0') << status.Pgp1FifoErr;
            cout <<                  hex << setw(1) << setfill('0') << status.Pgp0FifoErr;
            cout << endl;
         }
         else cout << "." << flush;

         // Forward packet
         pgpCardTx.pgpLane = (pgpCardRx.pgpLane == 0)?1:0;
         pgpCardTx.pgpVc   = pgpCardRx.pgpVc;
         pgpCardTx.size    = pgpCardRx.rxSize;
         write (s,&pgpCardTx,sizeof(PgpCardTx));
      }
   }

   close(s);
   return(0);
}
