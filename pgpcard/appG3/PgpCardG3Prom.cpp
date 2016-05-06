//-----------------------------------------------------------------------------
// File          : PgpCardG3Prom.cpp
// Author        : Larry Ruckman  <ruckman@slac.stanford.edu>
// Created       : 03/19/2014
// Project       :  
//-----------------------------------------------------------------------------
// Description :
//    PgpCardG2 PROM C++ Class
//-----------------------------------------------------------------------------
// Copyright (c) 2014 by SLAC. All rights reserved.
// Proprietary and confidential to SLAC.
//-----------------------------------------------------------------------------
// Modification history :
// 03/19/2014: created
//-----------------------------------------------------------------------------

#include <sstream>
#include <string>
#include <iostream>
#include <string.h>
#include <stdlib.h>
#include <iomanip> 

#include "PgpCardG3Prom.h"
#include "McsRead.h"

using namespace std;

#define GEN3_PROM_VERSION  0xCEC83000
#define GEN3_MASK          (GEN3_PROM_VERSION >> 12)
#define PROM_BLOCK_SIZE    0x4000 // Assume the smallest block size of 16-kword/block
#define READ_MASK          0x80000000
#define PROM_SIZE          0x009481F6

// Configuration: Force default configurations
#define CONFIG_REG      0xFD4F

// Constructor
PgpCardG3Prom::PgpCardG3Prom (void volatile *mapStart, string pathToFile ) {   
   // Set the file path
   filePath = pathToFile;
   
   // Setup the register Mapping
   memVersion = mapStart;// firmware version
   mapData    = (void volatile *)((__u32)mapStart+0xC00);//write cmd/data bus
   mapAddress = (void volatile *)((__u32)mapStart+0xC04);//write/read address bus
   mapRead    = (void volatile *)((__u32)mapStart+0xC08);// read data bus
   
   // Setup the configuration Register
   writeToFlash(CONFIG_REG,0x60,0x03);
}

// Deconstructor
PgpCardG3Prom::~PgpCardG3Prom ( ) { 
}

//! Check for a valid firmware version  (true=valid firmware version)
bool PgpCardG3Prom::checkFirmwareVersion ( ) {
   __u32 firmwareVersion = *((__u32*)memVersion);
   __u32 PgpCardGen = firmwareVersion >> 12;

   cout << "*******************************************************************" << endl;
   cout << "Current Firmware Version on the FPGA: 0x" << hex << firmwareVersion << endl;

   if(PgpCardGen!=GEN3_MASK){
   cout << "*******************************************************************" << endl;
      cout << "Error: Not a generation 3 PGP card" << endl;
      return false;
   } else {
      return true;
   }
}

//! Check if file exist (true=exists)
bool PgpCardG3Prom::fileExist ( ) {
  ifstream ifile(filePath.c_str());
  return ifile;
}

//! Print Power Cycle Reminder
void PgpCardG3Prom::rebootReminder ( ) {
   cout << "\n\n\n\n\n";
   cout << "***************************************" << endl;
   cout << "***************************************" << endl;
   cout << "The new data written in the PROM has " << endl;
   cout << "has been loaded into the FPGA. " << endl<< endl;
   cout << "A reboot or power cycle is required " << endl;
   cout << "to re-enumerate the PCIe card." << endl;
   cout << "***************************************" << endl;
   cout << "***************************************" << endl;
   cout << "\n\n\n\n\n";
}

//! Erase the PROM
void PgpCardG3Prom::eraseBootProm ( ) {

   __u32 address = 0;
   double size = double(PROM_SIZE);

   cout << "*******************************************************************" << endl;   
   cout << "Starting Erasing ..." << endl; 
   while(address<=PROM_SIZE) {       
      // Print the status to screen
      cout << hex << "Erasing PROM from 0x" << address << " to 0x" << (address+PROM_BLOCK_SIZE-1);
      cout << setprecision(3) << " ( " << ((double(address))/size)*100 << " percent done )" << endl;      
      
      // execute the erase command
      eraseCommand(address);
      
      //increment the address pointer
      address += PROM_BLOCK_SIZE;
   }   
   cout << "Erasing completed" << endl;
}

//! Write the .mcs file to the PROM
bool PgpCardG3Prom::bufferedWriteBootProm ( ) {
   cout << "*******************************************************************" << endl;
   cout << "Starting Writing ..." << endl; 
   McsRead mcsReader;
   McsReadData mem;
   
   __u32 address = 0;  
   __u16 fileData;
   __u16 i;
   
   __u32 bufAddr[256];  
   __u16 bufData[256];   
   __u16 bufSize = 0;
   
   double size = double(PROM_SIZE);
   double percentage;
   double skim = 5.0; 
   bool   toggle = false;

   //check for valid file path
   if ( !mcsReader.open(filePath) ) {
      mcsReader.close();
      cout << "mcsReader.close() = file path error" << endl;
      return false;
   }  
   
   //reset the flags
   mem.endOfFile = false;      
   
   //read the entire mcs file
   while(!mem.endOfFile) {
   
      //read a line of the mcs file
      if (mcsReader.read(&mem)<0){
         cout << "mcsReader.close() = line read error" << endl;
         mcsReader.close();
         return false;
      }
      
      // Check if this is the upper or lower byte
      if(!toggle) {
         toggle = true;
         fileData = (__u16)mem.data;
      } else {
         toggle = false;
         fileData |= ((__u16)mem.data << 8);
         
         // Latch the values
         bufAddr[bufSize] = address;
         bufData[bufSize] = fileData;
         bufSize++;
         
         // Check if we need to send the buffer
         if(bufSize==256) {
            bufferedProgramCommand(bufAddr,bufData,bufSize);
            bufSize = 0;
         }

         address++;
         percentage = (((double)address)/size)*100;
         percentage *= 2.0;//factor of two from two 8-bit reads for every write 16 bit write
         if(percentage>=skim) {
            skim += 5.0;
            cout << "Writing the PROM: " << percentage << " percent done" << endl;
         }         
      }
   }
   
   // Check if we need to send the buffer
   if(bufSize != 0) {
      // Pad the end of the block with ones
      for(i=bufSize;i<256;i++){
         bufData[bufSize] = 0xFFFF;
      }
      // Send the last block program 
      bufferedProgramCommand(bufAddr,bufData,256);  
   }     
   
   mcsReader.close();   
   cout << "Writing completed" << endl;   
   return true;
}

//! Compare the .mcs file with the PROM (true=matches)
bool PgpCardG3Prom::verifyBootProm ( ) {
   cout << "*******************************************************************" << endl;
   cout << "Starting Verification ..." << endl; 
   McsRead mcsReader;
   McsReadData mem;
   
   __u32 address = 0;  
   __u16 promData,fileData;
   double size = double(PROM_SIZE);
   double percentage;
   double skim = 5.0; 
   bool   toggle = false;

   //check for valid file path
   if ( !mcsReader.open(filePath) ) {
      mcsReader.close();
      cout << "mcsReader.close() = file path error" << endl;
      return(1);
   }  
   
   //reset the flags
   mem.endOfFile = false;   

   //read the entire mcs file
   while(!mem.endOfFile) {
   
      //read a line of the mcs file
      if (mcsReader.read(&mem)<0){
         cout << "mcsReader.close() = line read error" << endl;
         mcsReader.close();
         return false;
      }
      
      // Check if this is the upper or lower byte
      if(!toggle) {
         toggle = true;
         fileData = (__u16)mem.data;
      } else {
         toggle = false;
         fileData |= ((__u16)mem.data << 8);
         promData = readWordCommand(address);                
         if(fileData != promData) {
            cout << "verifyBootProm error = ";
            cout << "invalid read back" <<  endl;
            cout << hex << "\taddress: 0x"  << address << endl;
            cout << hex << "\tfileData: 0x" << fileData << endl;
            cout << hex << "\tpromData: 0x" << promData << endl;
            mcsReader.close();
            return false;
         }
         address++;
         percentage = (((double)address)/size)*100;
         percentage *= 2.0;//factore of two from two 8-bit reads for every write 16 bit write
         if(percentage>=skim) {
            skim += 5.0;
            cout << "Verifying the PROM: " << percentage << " percent done" << endl;
         }         
      }
   }
   
   mcsReader.close();  
   cout << "Verification completed" << endl;
   cout << "*******************************************************************" << endl;   
   return true;
}

//! Erase Command
void PgpCardG3Prom::eraseCommand(__u32 address) {
   __u16 status = 0;
   
   // Unlock the Block
   writeToFlash(address,0x60,0xD0);
   
   // Reset the status register
   writeToFlash(address,0x50,0x50);   
   
   // Send the erase command
   writeToFlash(address,0x20,0xD0);
   
   while(1) {
      // Get the status register
      status = readFlash(address,0x70);
      
      // Check for erasing failure
      if ( (status&0x20) != 0 ) {
      
         // Unlock the Block
         writeToFlash(address,0x60,0xD0);
         
         // Reset the status register
         writeToFlash(address,0x50,0x50);   
         
         // Send the erase command
         writeToFlash(address,0x20,0xD0);      
      
      // Check for FLASH not busy
      } else if ( (status&0x80) != 0 ) {
         break;
      }
   } 

   // Lock the Block
   writeToFlash(address,0x60,0x01);   
}

//! Program Command
void PgpCardG3Prom::programCommand(__u32 address, __u16 data) {
   __u16 status = 0;
   
   // Unlock the Block
   writeToFlash(address,0x60,0xD0);
   
   // Reset the status register
   writeToFlash(address,0x50,0x50);   
   
   // Send the program command
   writeToFlash(address,0x40,data);   
   
   while(1) {
      // Get the status register
      status = readFlash(address,0x70);
      
      // Check for programming failure
      if ( (status&0x10) != 0 ) {
      
         // Unlock the Block
         writeToFlash(address,0x60,0xD0);
         
         // Reset the status register
         writeToFlash(address,0x50,0x50);   
         
         // Send the program command
         writeToFlash(address,0x40,data);     
      
      // Check for FLASH not busy
      } else if ( (status&0x80) != 0 ) {
         break;
      }
   } 

   // Lock the Block
   writeToFlash(address,0x60,0x01);   
}

//! Buffered Program Command
void PgpCardG3Prom::bufferedProgramCommand(__u32 *address, __u16 *data, __u16 size) {
   __u16 status = 0;
   __u16 i;
   
   // Unlock the Block
   writeToFlash(address[0],0x60,0xD0);
   
   // Reset the status register
   writeToFlash(address[0],0x50,0x50);

   // Send the buffer program command and size
   writeToFlash(address[0],0xE8,(size-1));   
   
   // Load the buffer
   for(i=0;i<size;i++) {
      readFlash(address[i],data[i]);
   }
  
   // Confirm buffer programming
   readFlash(address[0],0xD0);  
   
   while(1) {
      // Get the status register
      status = readFlash(address[0],0x70);
      
      // Check for programming failure
      if ( (status&0x10) != 0 ) {
      
         // Unlock the Block
         writeToFlash(address[0],0x60,0xD0);
         
         // Reset the status register
         writeToFlash(address[0],0x50,0x50);   
         
         // Send the buffer program command and size
         writeToFlash(address[0],0xE8,(size-1));   
         
         // Load the buffer
         for(i=0;i<size;i++) {
            readFlash(address[i],data[i]);
         }
        
         // Confirm buffer programming
         readFlash(address[0],0xD0);                    
      
      // Check for FLASH not busy
      } else if ( (status&0x80) != 0 ) {
         break;
      }
   } 

   // Lock the Block
   writeToFlash(address[0],0x60,0x01);   
}

//! Read FLASH memory Command
__u16 PgpCardG3Prom::readWordCommand(__u32 address) {
   return readFlash(address,0xFF);
}

//! Generate request word 
__u32 PgpCardG3Prom::genReqWord(__u16 cmd, __u16 data) {
   __u32 readReq;
   readReq = ( ((__u32)cmd << 16) | ((__u32)data) );
   return readReq;
}

//! Generic FLASH write Command 
void PgpCardG3Prom::writeToFlash(__u32 address, __u16 cmd, __u16 data) {
   // Set the data bus
   *((__u32*)mapData) = genReqWord(cmd,data);
   
   // Set the address bus and initiate the transfer
   *((__u32*)mapAddress) = (~READ_MASK & address);
}

//! Generic FLASH read Command
__u16 PgpCardG3Prom::readFlash(__u32 address, __u16 cmd) {
   __u32 readReg;
      
   // Set the data bus
   *((__u32*)mapData) = genReqWord(cmd,0xFF);
   
   // Set the address bus and initiate the transfer
   *((__u32*)mapAddress) = (READ_MASK | address);   
   
   // Read the data register
   readReg = *((__u32*)mapRead);
   
   // return the readout data
   return (__u16)(readReg&0xFFFF);
}
