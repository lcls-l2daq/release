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
   void volatile *reboot;
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
   
   // Mapping the reboot register
   reboot = (void volatile *)((uint64_t)mapStart+0x01C);
   
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

   // Start the FPGA's 1 second reboot timer
   *((__u32*)reboot) = 0xBABECAFE;   
   
	// Close all the devices
   delete prom;
   close(fd);   
   return(0);
}
