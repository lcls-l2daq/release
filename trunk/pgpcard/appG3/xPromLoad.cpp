//////////////////////////////////////////////////////////////////////////////
// This file is part of 'SLAC PGP Gen3 Card'.
// It is subject to the license terms in the LICENSE.txt file found in the 
// top-level directory of this distribution and at: 
//    https://confluence.slac.stanford.edu/display/ppareg/LICENSE.html. 
// No part of 'SLAC PGP Gen3 Card', including this file, 
// may be copied, modified, propagated, or distributed except according to 
// the terms contained in the LICENSE.txt file.
//////////////////////////////////////////////////////////////////////////////
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <linux/types.h>

#include <fcntl.h>
#include <sstream>
#include <string>
#include <iomanip>
#include <iostream>
#include <string.h>
#include <stdlib.h>

#include "PgpCardG3Prom.h"

using namespace std;

#define PAGE_SIZE sysconf(_SC_PAGE_SIZE)

int main (int argc, char **argv) {

   int fd;
   void volatile *mapStart;
   PgpCardG3Prom *prom;
   string filePath;
   string devName = "/dev/PgpCardG3_0";

   // Check argc
   if ( argc < 2 ) {
      cout << "Usage: " << argv[0] << " filePath [deviceName]" << endl;
      return(0);
   } else {
      filePath = argv[1];
   }
   
   if (argc > 2) {
      devName = argv[2];
   }
	// Open the PCIe device
   if ( (fd = open(devName.c_str(), (O_RDWR|O_SYNC)) ) <= 0 ) {
      cout << "Error opening " << devName << endl;
      close(fd);
      return(1);
   }
      
   // Map the PCIe device from Kernel to Userspace
   mapStart = (void volatile *)mmap(NULL, PAGE_SIZE, (PROT_READ|PROT_WRITE), (MAP_SHARED|MAP_LOCKED), fd, 0);   
   if(mapStart == MAP_FAILED){
      cout << "Error: mmap() = " << dec << mapStart << endl;
      close(fd);
      return(1);   
   }
   
   // Create the PgpCardG3Prom object
   prom = new PgpCardG3Prom(mapStart,filePath);
   
   // Check if the .mcs file exists
   if(!prom->fileExist()){
      cout << "Error opening: " << filePath << endl;
      delete prom;
      close(fd);
      return(1);   
   }   
   
   // Check if the PCIe device is a generation 2 card
   if(!prom->checkFirmwareVersion()){
      delete prom;
      close(fd);
      return(1);   
   }    
      
   // Erase the PROM
   prom->eraseBootProm();
  
   // Write the .mcs file to the PROM
   if(!prom->bufferedWriteBootProm()) {
      cout << "Error in prom->bufferedWriteBootProm() function" << endl;
      delete prom;
      close(fd);
      return(1);     
   }   

   // Compare the .mcs file with the PROM
   if(!prom->verifyBootProm()) {
      cout << "Error in prom->verifyBootProm() function" << endl;
      delete prom;
      close(fd);
      return(1);     
   }
      
   // Display Reminder
   prom->rebootReminder();
   
	// Close all the devices
   delete prom;
   close(fd);   
   return(0);
}
