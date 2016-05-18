#include <sys/types.h>
#include <linux/types.h>
#include <unistd.h>
#include <stdio.h>
#include <termios.h>
#include <fcntl.h>
#include <sstream>
#include <time.h>
#include <string>
#include <iomanip>
#include <iostream>
#include <pthread.h>
#include "../include/PgpCardMod.h"
using namespace std;

#define DEVNAME "/dev/pgpcard0"

#define TX_SIZE 500000
//#define TX_SIZE 5000
//#define TX_SIZE 500
//#define TX_SIZE 200
//#define TX_SIZE 18

class RunData {
   public:
      int fd;
      unsigned long count;
      unsigned long total;
};


void *runWrite ( void *t ) {
   fd_set          fds;
   struct timeval  timeout;
   uint            error;
   int             ret;
   uint *          data;
   uint            size;
   uint            lane;
   uint            vc;

   RunData *txData = (RunData *)t;

   size = TX_SIZE;
   data = (uint *)malloc(sizeof(uint)*size);
   lane = 0;
   vc   = 0;

   cout << "Starting write thread" << endl;

   error = 0;
   while (error == 0) {

      // Setup fds for select call
      FD_ZERO(&fds);
      FD_SET(txData->fd,&fds);

      // Wait for write ready
      timeout.tv_sec=5;
      timeout.tv_usec=0;
      ret = select(txData->fd+1,NULL,&fds,NULL,&timeout);
      if ( ret <= 0 ) {
         cout << "Write timeout. Ret=" << ret << endl;
         error++;
      }
      else {
         ret = pgpcard_send (txData->fd,data,size,lane,vc);
         if ( ret <= 0 ) {
            cout << "Write Error" << endl;
            error++;
         }
         else {
            txData->count++;
            txData->total += ret;
         }
         lane = (lane + 1) % 4;
      }
   }
   free(data);
   pthread_exit(NULL);
}


void *runRead ( void *t ) {
   fd_set          fds;
   struct timeval  timeout;
   uint            error;
   int             ret;
   uint *          data;
   uint            maxSize;
   uint            lane;
   uint            vc;
   uint            eofe;
   uint            fifoErr;
   uint            lengthErr;

   RunData *rxData = (RunData *)t;

   maxSize = TX_SIZE*2;
   data = (uint *)malloc(sizeof(uint)*maxSize);

   cout << "Starting read thread" << endl;

   error = 0;
   while (error == 0) {

      // Setup fds for select call
      FD_ZERO(&fds);
      FD_SET(rxData->fd,&fds);

      // Wait for read ready
      timeout.tv_sec=5;
      timeout.tv_usec=0;
      ret = select(rxData->fd+1,&fds,NULL,NULL,&timeout);
      if ( ret <= 0 ) {
         cout << "Read timeout. Ret=" << ret << endl;
         error++;
      }
      else {
         ret = pgpcard_recv (rxData->fd,data,maxSize,&lane,&vc,&eofe,&fifoErr,&lengthErr);
         if ( ret != TX_SIZE ) {
            cout << "Read Error. Ret=" << dec << ret << endl;
            error++;
         }
         else {
            rxData->count++;
            rxData->total += ret;
         }
      }
   }
   free (data);
   pthread_exit(NULL);
}


int main (int argc, char **argv) {
   RunData *txData = new RunData;
   RunData *rxData = new RunData;
   pthread_t rxThread;
   pthread_t txThread;
   int fd;
   int seconds;
   time_t c_tme;
   time_t l_tme;
   uint lastRx;
   uint lastTx;
   PgpCardStatus status;

   if ( (fd = open(DEVNAME, O_RDWR )) < 0 ) {
      cout << "Error opening File" << endl;
      return(1);
   }
   seconds       = 0;
   txData->fd    = fd;
   txData->count = 0;
   txData->total = 0;
   rxData->fd    = fd;
   rxData->count = 0;
   rxData->total = 0;

   time(&c_tme);    
   time(&l_tme);    

   if ( pthread_create(&txThread,NULL,runWrite,txData) ) {
      cout << "Error creating write thread" << endl;
      return(2);
   }
   if ( pthread_create(&rxThread,NULL,runRead,rxData) ) {
      cout << "Error creating read thread" << endl;
      return(2);
   }

   lastRx = 0;
   lastTx = 0;
   while (1) {
      sleep(1);
      time(&c_tme);
      cout << "Seconds=" << dec << seconds;
      cout << ", Rx Count=" << dec << rxData->count;
      cout << ", Rx Total=" << dec << rxData->total;
      cout << ", Rx Rate=" << ((double)(rxData->count-lastRx) * 32.0 * (double)TX_SIZE) / (double)(c_tme-l_tme);
      cout << ", Tx Count=" << dec << txData->count;
      cout << ", Tx Total=" << dec << txData->total;
      cout << ", Tx Rate=" << ((double)(txData->count-lastTx) * 32.0 * (double)TX_SIZE) / (double)(c_tme-l_tme);
      cout << endl;
      if ( seconds++ % 10 == 0 ) {
         pgpcard_status(fd, &status);
         cout << "        TxDma3AFull: " << hex << setw(1) << setfill('0') << status.TxDma3AFull << endl;
         cout << "        TxDma2AFull: " << hex << setw(1) << setfill('0') << status.TxDma2AFull << endl;
         cout << "        TxDma1AFull: " << hex << setw(1) << setfill('0') << status.TxDma1AFull << endl;
         cout << "        TxDma0AFull: " << hex << setw(1) << setfill('0') << status.TxDma0AFull << endl;
         cout << "        TxReadReady: " << hex << setw(1) << setfill('0') << status.TxReadReady << endl;
         cout << "     TxRetFifoCount: " << hex << setw(3) << setfill('0') << status.TxRetFifoCount << endl;
         cout << "            TxCount: " << hex << setw(8) << setfill('0') << status.TxCount << endl;
         cout << "            TxWrite: " << hex << setw(2) << setfill('0') << status.TxWrite << endl;
         cout << "             TxRead: " << hex << setw(2) << setfill('0') << status.TxRead  << endl;
         cout << "        RxFreeEmpty: " << hex << setw(1) << setfill('0') << status.RxFreeEmpty << endl;
         cout << "         RxFreeFull: " << hex << setw(1) << setfill('0') << status.RxFreeFull << endl;
         cout << "        RxFreeValid: " << hex << setw(1) << setfill('0') << status.RxFreeValid << endl;
         cout << "    RxFreeFifoCount: " << hex << setw(3) << setfill('0') << status.RxFreeFifoCount << endl;
         cout << "        RxReadReady: " << hex << setw(1) << setfill('0') << status.RxReadReady << endl;
         cout << "     RxRetFifoCount: " << hex << setw(3) << setfill('0') << status.RxRetFifoCount << endl;
         cout << "            RxCount: " << hex << setw(8) << setfill('0') << status.RxCount << endl;
         cout << "            RxWrite: " << hex << setw(2) << setfill('0') << status.RxWrite << endl;
         cout << "             RxRead: " << hex << setw(2) << setfill('0') << status.RxRead  << endl;
         cout << "     Pgp0CellErrCnt: " << hex << setw(1) << setfill('0') << status.Pgp3CellErrCnt;
         cout <<                     " " << hex << setw(1) << setfill('0') << status.Pgp2CellErrCnt;
         cout <<                     " " << hex << setw(1) << setfill('0') << status.Pgp1CellErrCnt;
         cout <<                     " " << hex << setw(1) << setfill('0') << status.Pgp0CellErrCnt << endl;
         cout << "    Pgp0LinkDownCnt: " << hex << setw(1) << setfill('0') << status.Pgp3LinkDownCnt;
         cout <<                     " " << hex << setw(1) << setfill('0') << status.Pgp2LinkDownCnt;
         cout <<                     " " << hex << setw(1) << setfill('0') << status.Pgp1LinkDownCnt;
         cout <<                     " " << hex << setw(1) << setfill('0') << status.Pgp0LinkDownCnt << endl;
         cout << "     Pgp0LinkErrCnt: " << hex << setw(1) << setfill('0') << status.Pgp3LinkErrCnt;
         cout <<                     " " << hex << setw(1) << setfill('0') << status.Pgp2LinkErrCnt;
         cout <<                     " " << hex << setw(1) << setfill('0') << status.Pgp1LinkErrCnt;
         cout <<                     " " << hex << setw(1) << setfill('0') << status.Pgp0LinkErrCnt << endl;
         cout << "        Pgp0FifoErr: " << hex << setw(1) << setfill('0') << status.Pgp3FifoErr;
         cout <<                     " " << hex << setw(1) << setfill('0') << status.Pgp2FifoErr;
         cout <<                     " " << hex << setw(1) << setfill('0') << status.Pgp1FifoErr;
         cout <<                     " " << hex << setw(1) << setfill('0') << status.Pgp0FifoErr << endl;
      }
      lastRx = rxData->count;
      lastTx = txData->count;
      l_tme = c_tme;
   }

   // Wait for thread to stop
   pthread_join(txThread, NULL);
   pthread_join(rxThread, NULL);

   close(fd);
   return(0);
}
