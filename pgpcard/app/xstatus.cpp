
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
#include "../include/PgpCardStatus.h"
#define NUMBER_OF_LANES 4
using namespace std;

int main (int argc, char **argv) {
   PgpCardStatus status;
   int           fd;
   int           ret;
   int           i;

   if ( argc != 2 ) {
      cout << "Usage: xstatus device" << endl;
      return(0);
   }

   if ( (fd = open(argv[1], O_RDWR)) <= 0 ) {
      cout << "Error opening " << argv[1] << endl;
      return(1);
   }

   memset(&status,0,sizeof(PgpCardStatus));
   // the buffer is a PgpCardTx on the way in and a PgpCardStatus on the way out
   __u8*      c = (__u8*) &status;  // this adheres to strict aliasing rules
   PgpCardTx* p = (PgpCardTx*) c;

   p->model = sizeof(p);
   p->cmd   = IOCTL_Read_Status;
   p->data  = (__u32*)&status;
   ret = write(fd, p, sizeof(PgpCardTx));

   cout << "PGP Card Status:      0 1 2 3 <-- Lane order" << endl;
   cout << "            Version: " << hex << setw(8) << setfill('0') << status.Version << endl;
   cout << "         ScratchPad: " << hex << setw(8) << setfill('0') << status.ScratchPad << endl;
   cout << "         PciCommand: " << hex << setw(4) << setfill('0') << status.PciCommand << endl;
   cout << "          PciStatus: " << hex << setw(4) << setfill('0') << status.PciStatus << endl;
   cout << "        PciDCommand: " << hex << setw(4) << setfill('0') << status.PciDCommand << endl;
   cout << "         PciDStatus: " << hex << setw(4) << setfill('0') << status.PciDStatus << endl;
   cout << "        PciLCommand: " << hex << setw(4) << setfill('0') << status.PciLCommand << endl;
   cout << "         PciLStatus: " << hex << setw(4) << setfill('0') << status.PciLStatus << endl;
   cout << "Negotiated Lnk Wdth: " << dec << setw(1) << ((status.PciLStatus >> 4) & 0x3f) << endl;
   cout << "       PciLinkState: " << hex << setw(1) << setfill('0') << status.PciLinkState << endl;
   cout << "        PciFunction: " << hex << setw(1) << setfill('0') << status.PciFunction << endl;
   cout << "          PciDevice: " << hex << setw(1) << setfill('0') << status.PciDevice << endl;
   cout << "             PciBus: " << hex << setw(2) << setfill('0') << status.PciBus << endl;
   cout << "       Pgp0LoopBack: " ;
   for (i=0; i<NUMBER_OF_LANES; i++ ) {
     cout << hex << setw(1) << status.PgpLink[i].PgpLoopBack << " ";
   }
   cout <<  endl << "        Pgp0RxReset: " ;
   for (i=0; i<NUMBER_OF_LANES; i++ ) {
     cout << hex << setw(1) << status.PgpLink[i].PgpRxReset << " ";
   }
   cout << endl << "        Pgp0TxReset: " ;
   for (i=0; i<NUMBER_OF_LANES; i++ ) {
     cout << hex << setw(1) << status.PgpLink[i].PgpTxReset << " ";
   }
   cout << endl << "   Pgp0LocLinkReady: " ;
   for (i=0; i<NUMBER_OF_LANES; i++ ) {
     cout << hex << setw(1) << status.PgpLink[i].PgpLocLinkReady << " ";
   }
   cout << endl << "   Pgp0RemLinkReady: " ;
   for (i=0; i<NUMBER_OF_LANES; i++ ) {
     cout << hex << setw(1) << status.PgpLink[i].PgpRemLinkReady << " ";
   }
   cout << endl << "        Pgp0RxReady: " ;
   for (i=0; i<NUMBER_OF_LANES; i++ ) {
     cout << hex << setw(1) << status.PgpLink[i].PgpRxReady << " ";
   }
   cout << endl << "        Pgp0TxReady: " ;
   for (i=0; i<NUMBER_OF_LANES; i++ ) {
     cout << hex << setw(1) << status.PgpLink[i].PgpTxReady << " ";
   }
   cout << endl << "        Pgp0RxCount: " ;
   for (i=0; i<NUMBER_OF_LANES; i++ ) {
     cout << hex << setw(1) << status.PgpLink[i].PgpRxCount << " ";
   }
   cout << endl << "     Pgp0CellErrCnt: " ;
   for (i=0; i<NUMBER_OF_LANES; i++ ) {
     cout << hex << setw(1) << status.PgpLink[i].PgpCellErrCnt << " ";
   }
   cout << endl << "    Pgp0LinkDownCnt: " ;
   for (i=0; i<NUMBER_OF_LANES; i++ ) {
     cout << hex << setw(1) << status.PgpLink[i].PgpLinkDownCnt << " ";
   }
   cout << endl << "     Pgp0LinkErrCnt: " ;
   for (i=0; i<NUMBER_OF_LANES; i++ ) {
     cout << hex << setw(1) << status.PgpLink[i].PgpLinkErrCnt << " ";
   }
   cout << endl << "        Pgp0FifoErr: " ;
   for (i=0; i<NUMBER_OF_LANES; i++ ) {
     cout << hex << setw(1) << status.PgpLink[i].PgpFifoErr << " ";
   }
   cout << endl;
   cout << "        TxDma3AFull: " << hex << setw(1) << setfill('0') << status.TxDma3AFull << endl;
   cout << "        TxDma2AFull: " << hex << setw(1) << setfill('0') << status.TxDma2AFull << endl;
   cout << "        TxDma1AFull: " << hex << setw(1) << setfill('0') << status.TxDma1AFull << endl;
   cout << "        TxDma0AFull: " << hex << setw(1) << setfill('0') << status.TxDma0AFull << endl;
   cout << "        TxReadReady: " << hex << setw(1) << setfill('0') << status.TxReadReady << endl;
   cout << "     TxRetFifoCount: " << hex << setw(3) << setfill('0') << status.TxRetFifoCount << endl;
   cout << "            TxCount: " << hex << setw(8) << setfill('0') << status.TxCount << endl;
   cout << "      TxBufferCount: " << hex << setw(2) << setfill('0') << status.TxBufferCount << endl;
   cout << "             TxRead: " << hex << setw(2) << setfill('0') << status.TxRead  << endl;
   cout << "        RxFreeEmpty: " << hex << setw(1) << setfill('0') << status.RxFreeEmpty << endl;
   cout << "         RxFreeFull: " << hex << setw(1) << setfill('0') << status.RxFreeFull << endl;
   cout << "        RxFreeValid: " << hex << setw(1) << setfill('0') << status.RxFreeValid << endl;
   cout << "    RxFreeFifoCount: " << hex << setw(3) << setfill('0') << status.RxFreeFifoCount << endl;
   cout << "        RxReadReady: " << hex << setw(1) << setfill('0') << status.RxReadReady << endl;
   cout << "     RxRetFifoCount: " << hex << setw(3) << setfill('0') << status.RxRetFifoCount << endl;
   cout << "            RxCount: " << hex << setw(8) << setfill('0') << status.RxCount << endl;
   cout << "      RxBufferCount: " << hex << setw(2) << setfill('0') << status.RxBufferCount << endl << "    RxWrite[client]: " ;
   for (i=0; i<NUMBER_OF_LANES; i++ ) {
     cout << hex << setw(2) << status.RxWrite[i] << " ";
   }
   cout << endl << "     RxRead[client]: ";
   for (i=0; i<NUMBER_OF_LANES; i++ ) {
     cout << hex << setw(2) << status.RxRead[i] << " ";
   }
   cout << endl;
   cout << endl;

   p->model = sizeof(p);
   p->cmd   = IOCTL_Dump_Debug;
   p->data  = (__u32*)0;
   write(fd, p, sizeof(PgpCardTx));

   close(fd);
}
